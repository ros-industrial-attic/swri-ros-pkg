//vfh_recognition_node.cpp
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include "template_alignment.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/ModelCoefficients.h>
#include "euclidean_segmentation.h"
#include "vfh_recognition/Recognize.h"

//Usage: ./build/nearest_neighbors_raw -k 16 -thresh 50 ~/pcl_workspace/euclidean_cluster/cloud_cluster_1.pcd

typedef std::pair<std::string, std::vector<float> > vfh_model;
std::vector<vfh_model> models;
flann::Matrix<int> k_indices;
flann::Matrix<float> k_distances;
flann::Matrix<float> data;
ros::Publisher recognized_pub;

/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  std::ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

bool recognize_cb(vfh_recognition::Recognize::Request &fromKinect,
		  vfh_recognition::Recognize::Response &res)
{
  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  
  //Euclidean segmentation:
  SegmentCloud(fromKinect.scene, clouds);
 
  //For storing results:
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_template (new pcl::PointCloud<pcl::PointXYZ>);
  bool objectFound = false;
  sensor_msgs::PointCloud2 recognized_msg;
  Eigen::Matrix4f objectToView;
  
  //For each segment passed in:
  for(unsigned int segment_it = 0; segment_it < clouds.size(); segment_it++){
    vfh.setInputCloud (clouds.at(segment_it));
    //Estimate normals:
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (clouds.at(segment_it));
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNorm (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (treeNorm);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
    
    //VFH estimation
    vfh.setInputNormals (cloud_normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod (tree);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    vfh.compute (*vfhs);

    //Load histogram  
    vfh_model histogram;
    histogram.second.resize(308);
    for (size_t i = 0; i < 308; ++i)
      {
	histogram.second[i] = vfhs->points[0].histogram[i];
      }

    //Algorithm parameters  
    float thresh = 125; //similarity threshold
    int k = 1; //number of neighbors
    
    //KNN classification
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);

    // Output the results on screen
    //pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, models.at(k_indices[0][0].first));
    for (int i = 0; i < k; ++i)
      pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
	  i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);

    //Once a model is matched, do finer pose estimation by RANSAC fitting.
    if(k_distances[0][0] < thresh){
      objectFound = true;
      //std::cout << "OBJECT FOUND!!!" << std::endl;
      //Load nearest match
      std::string cloud_name = models.at(k_indices[0][0]).first;
      cloud_name.erase(cloud_name.end()-8, cloud_name.end()-4);
      
      std::string recognitionLabel, viewNumber;
      recognitionLabel.assign(cloud_name.begin()+5, cloud_name.end()-6);
      viewNumber.assign(cloud_name.end()-5, cloud_name.end()-4);
      //std::cout << "Label: " << recognitionLabel << std::endl << "View Number: " << viewNumber << std::endl;
      
      //ROS_INFO the recognitionLabel and view number. This info will later be used to look the object up in a manipulation database.
      ROS_INFO("%s", recognitionLabel.c_str());
      ROS_INFO("%i", atoi(viewNumber.c_str()));
      
      //Here, objectToView is the transformation of the detected object to it's nearest viewpoint in the database.
      //To get the object pose in the world frame: T(camera_to_world)*T(training_view_to_camera)*objectToView 
      alignTemplate(clouds.at(segment_it), cloud_name, aligned_template, objectToView);
      printf ("\n");
      printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2)), rotation (0,3);
      printf ("T = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2)), rotation (1,3);
      printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2)), rotation (2,3);
      printf ("    | %6.3f %6.3f %6.3f | \n", rotation (3,0), rotation (3,1), rotation (3,2)), rotation (3,3);
      printf ("\n");
      
      pcl::toROSMsg(*aligned_template, res.model);
      
    }else{   
     //std::cout << "Object not found." << std::endl;
    }
    
  }
  if(!objectFound){
      aligned_template->clear();
      pcl::toROSMsg(*aligned_template, res.model);
    }
  return(1);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "recongition_node");
  ros::NodeHandle n;
  std::string kdtree_idx_file_name = "kdtree.idx";
  std::string training_data_h5_file_name = "training_data.h5";
  std::string training_data_list_file_name = "training_data.list";
  
  loadFileList (models, training_data_list_file_name);
  flann::load_from_file (data, training_data_h5_file_name, "training_data");
  pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
  (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
  
  ros::ServiceServer serv = n.advertiseService("/object_recognition", recognize_cb);
  ROS_INFO("Recognition node ready.");
  ros::spin();

  
}
