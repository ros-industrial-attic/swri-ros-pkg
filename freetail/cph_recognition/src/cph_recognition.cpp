//cph_recognition_node.cpp
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "euclidean_segmentation.h"
#include "template_alignment.h"
#include "cph.h"



typedef std::pair<std::string, std::vector<float> > cph_model;
std::vector<cph_model> models;
flann::Matrix<int> k_indices;
flann::Matrix<float> k_distances;
flann::Matrix<float> *data;
sensor_msgs::PointCloud2 fromKinect;
ros::Publisher pub;
std::ofstream outFile;
int segCount = 0;
int num_ybins = 3; int num_rbins = 36;
int histSize = num_ybins*num_rbins+3;


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const cph_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second.at(0), p.cols * p.rows * sizeof (int));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

bool
loadHist (const boost::filesystem::path &path, cph_model &cph)
{
  //path is the location of the file being read.
  std::ifstream featureFile;
  featureFile.open(path.string().c_str(), std::ifstream::in);
  cph.second.clear();
  float value;
  for(unsigned int i=0; i<histSize; i++){
   featureFile >> value;
   cph.second.push_back(value); 
  }
  cph.first = path.string ();
  
  return (true);
}

void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                   std::vector<cph_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadFeatureModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      cph_model m;
      if (loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}


bool recognize_cb(tabletop_object_detector::TabletopObjectRecognition::Request &srv_request,
		  tabletop_object_detector::TabletopObjectRecognition::Response &srv_response)
{
  
  
  //lookup table for model_id's
  std::map<std::string, int> classMap;
//   classMap.insert(std::pair<std::string, int>("bowl", 0));
//   classMap.insert(std::pair<std::string, int>("camera", 1));
//   classMap.insert(std::pair<std::string, int>("coffee_mug", 2));
//   classMap.insert(std::pair<std::string, int>("dry_battery", 3));
//   classMap.insert(std::pair<std::string, int>("flashlight", 4));
//   classMap.insert(std::pair<std::string, int>("food_jar", 5));
//   classMap.insert(std::pair<std::string, int>("pliers", 6));
  
  // Build the tree index and save it to disk
  //pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", (int)data.rows);
  flann::Index<flann::ChiSquareDistance<float> > index (*data, flann::LinearIndexParams ());
  index.buildIndex ();
  
    //clear any models in the response:
  srv_response.models.resize(0);
  
  // Create cph estimation.
  //std::cout << "Creating cph estimation object... ";
  CPHEstimation2 cph(num_ybins,num_rbins);
  //std::cout << "done.\n";
  
  //Euclidean segmentation:
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  SegmentCloud(fromKinect, clouds);
  
  std::cout << "Found " << clouds.size() << "segments.\n";
  
  //For storing results:
   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_template (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 recognized_msg;
  Eigen::Matrix4f objectToView;
  int numFound = 0;
  
  
  //For each test file:
  for(unsigned int segment_it = 0; segment_it < clouds.size(); segment_it++){
    
    //Hold results:
    std::string label, angleStr;
    int angle;
    Eigen::Vector4f translation;
      
    //Demean the cloud.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*clouds.at(segment_it), centroid);
    
    //Compute cph:
    cph.setInputCloud (clouds.at(segment_it));
    std::vector<float> feature;
    //std::cout << "computing feature... ";
    cph.compute(feature);
    //std::cout << "done.\n";
 
    //Algorithm parameters  
    float thresh = 280; //similarity threshold
    int k = 1; //number of neighbors
    
    cph_model histogram;
    histogram.second.resize(histSize);
    
    
    //std::cout << "copying feature to histogram... ";
    for (size_t i = 0; i < histSize; ++i)
    {
	histogram.second[i] = feature.at(i);
    }
    //std::cout << "done.\n";
    //KNN classification
    nearestKSearch (index, histogram, k, k_indices, k_distances);
    //determine label and pose:
    
    //std::cout << "distance: " << k_distances[0][0] << std::endl; 
    if(k_distances[0][0] < thresh){
      std::cout << "distance: " << k_distances[0][0] << std::endl;
      ROS_INFO("%f", k_distances[0][0]);
      numFound++;
      //Load nearest match
      std::string cloud_name = models.at(k_indices[0][0]).first;
      cloud_name.replace(cloud_name.end()-3, cloud_name.end(), "pcd");
      pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud (new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::io::loadPCDFile (cloud_name, *modelCloud);
//       Eigen::Vector4f modelCentroid;
//       pcl::compute3DCentroid (*modelCloud, modelCentroid);
//       translation = centroid-modelCentroid;
      label.assign(cloud_name.begin()+5, cloud_name.begin()+cloud_name.rfind("_"));
      angleStr.assign(cloud_name.begin()+cloud_name.rfind("_")+1, cloud_name.end()-4);
      angle = atoi(angleStr.c_str());
      translation(3) = angle;
      std::cout << "done.\n";
      ROS_INFO("%s", label.c_str());
      ROS_INFO("%i", angle);
    
      alignTemplate(clouds.at(segment_it), cloud_name, aligned_template, objectToView);
      Eigen::Matrix3f rotation = objectToView.block<3,3>(0, 0);
      Eigen::Quaternionf rotQ(rotation);
      
      //Build service response:
      srv_response.models.resize(numFound);
      srv_response.models[numFound-1].model_list.resize(1);
      //model_id
      srv_response.models[numFound-1].model_list[0].model_id = angle; //or classMap[label]
      //PoseStamped
	//Header
      srv_response.models[numFound-1].model_list[0].pose.header.seq = 1; //Don't know what this is, but it's set.
      srv_response.models[numFound-1].model_list[0].pose.header.stamp = fromKinect.header.stamp;
      srv_response.models[numFound-1].model_list[0].pose.header.frame_id = "/camera_depth_optical_frame";
	//Pose
	  //Position:
      srv_response.models[numFound-1].model_list[0].pose.pose.position.x = objectToView(0,3);//translation(0); 
      srv_response.models[numFound-1].model_list[0].pose.pose.position.y = objectToView(1,3);//translation(1);
      srv_response.models[numFound-1].model_list[0].pose.pose.position.z = objectToView(2,3);//translation(2);
	  //Orientation:
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.x = rotQ.x();//Fill with parameters from a lookuptable for angle.
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.y = rotQ.y();//ex:rotQ = poseMap[angle]
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.z = rotQ.z();//where poseMap is a map<int,Eigen::Quaternionf>
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.w = rotQ.w();//OR just compute rotQ from the angle if the rotation axis (vertical) is known.
      //confidence and cluster_model_indcices are not currently used
    }
        
  }
  return(1);
}

void kinect_cb(const sensor_msgs::PointCloud2 inCloud)
{
  fromKinect = inCloud;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "recongition_node");
  ros::NodeHandle n;
  
  loadFeatureModels ("data", ".csv", models);
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data\n", 
      (int)models.size ());

  // Convert data into FLANN format
  data = new flann::Matrix<float> (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
  
  std::cout << "data size: [" << data->rows << " , " << data->cols << "]\n";
  for (size_t i = 0; i < data->rows; ++i)
    for (size_t j = 0; j < data->cols; ++j)
      *(data->ptr()+(i*data->cols + j)) = models[i].second[j];
    
  pcl::console::print_error ("Training data loaded.\n");
  
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  
  ros::ServiceServer serv = n.advertiseService("/object_recognition", recognize_cb);
  
  ROS_INFO("Recognition node ready.");
  
//   outFile.open("segment_matches.txt");
//   outFile << "segment , model, distance\n";
  ros::spin();

  
}

