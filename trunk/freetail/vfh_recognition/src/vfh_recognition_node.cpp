//vfh_recognition_node.cpp
#define BOOST_FILESYSTEM_VERSION 2

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
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/ModelCoefficients.h>
#include "vfh_recognition/Recognize.h"
#include <vfh_recognition/SupportClasses.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <boost/filesystem.hpp>



typedef std::pair<std::string, std::vector<float> > vfh_model;
std::vector<vfh_model> models;
flann::Matrix<int> k_indices;
flann::Matrix<float> k_distances;
flann::Matrix<float> *data;
//ros::Publisher recognized_pub;
sensor_msgs::PointCloud2 fromKinect;
ros::Publisher pub;

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

bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    sensor_msgs::PointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type;  int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (pcl::InvalidConversionException e)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <sensor_msgs::PointField> fields;
  pcl::getFieldIndex (point, "vfh", fields);

  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}

void
loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                   std::vector<vfh_model> &models)
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
      vfh_model m;
      if (loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}

bool recognize_cb(tabletop_object_detector::TabletopObjectRecognition::Request &srv_request,
		  tabletop_object_detector::TabletopObjectRecognition::Response &srv_response)
{
  
  //build kdtree index
  flann::Index<flann::ChiSquareDistance<float> > index (*data, flann::LinearIndexParams ());
   index.buildIndex ();
   
  //clear any models in the response:
  srv_response.models.resize(0);
  
  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  
  //Euclidean segmentation:
  SegmentCloud(fromKinect, clouds);
 
  //For storing results:
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_template (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 recognized_msg;
  Eigen::Matrix4f objectToView;
  int numFound = 0;
  std::cout << "Found " << clouds.size() << " clusters.\n"; 
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
    float thresh = 100; //similarity threshold
    int k = 1; //number of neighbors
    
    //KNN classification
    flann::Index<flann::ChiSquareDistance<float> > index (*data, flann::LinearIndexParams ());
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);

    //If model match is close enough, do finer pose estimation by RANSAC fitting.
    if(k_distances[0][0] < thresh){
      numFound++;
      //Load nearest match
      std::string cloud_name = models.at(k_indices[0][0]).first;
      
      //Extract object label and view number from file name:
      cloud_name.erase(cloud_name.end()-8, cloud_name.end()-4);
      std::string recognitionLabel, viewNumber;
      recognitionLabel.assign(cloud_name.begin()+5, cloud_name.end()-6);
      viewNumber.assign(cloud_name.end()-5, cloud_name.end()-4);
    
      
      //ROS_INFO the recognitionLabel and view number. This info will later be used to look the object up in a manipulation database.
      ROS_INFO("%s", recognitionLabel.c_str());
      ROS_INFO("%i", atoi(viewNumber.c_str()));
      
      //Here, objectToView is the transformation of the detected object to its nearest viewpoint in the database.
      //To get the object pose in the world frame: T(camera_to_world)*T(training_view_to_camera)*objectToView. 
      //The transformation to camera coordinates would then happen here by finding T(view_cam) in a lookup table and premultiplying
      //by objectToView
      alignTemplate(clouds.at(segment_it), cloud_name, aligned_template, objectToView);
      //Convert rotational component of objectToView to Quaternion for messaging:
      Eigen::Matrix3f rotation = objectToView.block<3,3>(0, 0);
      Eigen::Quaternionf rotQ(rotation);
     
      //Set num models in request:
      srv_request.num_models = numFound;
      //Build service response:
      srv_response.models.resize(numFound);
      srv_response.models[numFound-1].model_list.resize(1);
      //model_id
      srv_response.models[numFound-1].model_list[0].model_id = atoi(viewNumber.c_str()); //This ID will eventually correspond to the object label.
      //PoseStamped
        //Header
      srv_response.models[numFound-1].model_list[0].pose.header.seq = 1; //Don't know what this is, but it's set.
      srv_response.models[numFound-1].model_list[0].pose.header.stamp = fromKinect.header.stamp;
      srv_response.models[numFound-1].model_list[0].pose.header.frame_id = "/camera_depth_optical_frame";
        //Pose
          //Position:
      srv_response.models[numFound-1].model_list[0].pose.pose.position.x = objectToView(0,3);
      srv_response.models[numFound-1].model_list[0].pose.pose.position.y = objectToView(1,3);
      srv_response.models[numFound-1].model_list[0].pose.pose.position.z = objectToView(2,3);
          //Orientation:
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.x = rotQ.x();
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.y = rotQ.y();
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.z = rotQ.z();
      srv_response.models[numFound-1].model_list[0].pose.pose.orientation.w = rotQ.w();
      //confidence and cluster_model_indcices are not currently used.
      
    }//end threshold if statement
    
  }//end segment iterator
  
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
  
  loadFeatureModels ("data", ".pcd", models);
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
  ros::spin();

  
}
