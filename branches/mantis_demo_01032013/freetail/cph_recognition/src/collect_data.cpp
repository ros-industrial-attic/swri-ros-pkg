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
#include "cph.h"



typedef std::pair<std::string, std::vector<float> > cph_model;
std::vector<cph_model> models;
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
  //clear any models in the response:
  srv_response.models.resize(0);
  
  // Create cph estimation.
  CPHEstimation2 cph(num_ybins,num_rbins);
  
  //Euclidean segmentation:
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  SegmentCloud(fromKinect, clouds);
  
  //For storing results:
  sensor_msgs::PointCloud2 recognized_msg;
  Eigen::Matrix4f objectToView;
  int numFound = 0;
  
  
  //For each test file:
  std::cout << "Found " << clouds.size() << " segments.\n";
  for(unsigned int segment_it = 0; segment_it < clouds.size(); segment_it++){
      
    //Compute cph:
    cph.setInputCloud (clouds.at(segment_it));
    std::vector<float> feature;
    //std::cout << "computing feature... ";
    cph.compute(feature);
    //std::cout << "done.\n";
    
    cph_model histogram;
    histogram.second.resize(histSize);
    
    for (size_t i = 0; i < histSize; ++i)
    {
	histogram.second[i] = feature.at(i);
    }
    
    std::cout << "Writing segment " << segCount << " to file.\n";
    //Save segment:
    std::stringstream segment_name("");
    segment_name << "match/segment_" << segCount << ".pcd";
    pcl::io::savePCDFile(segment_name.str().c_str(), *clouds.at(segment_it));
    numFound++;
    //Save feature:
    std::ofstream featureFile;
    std::stringstream featureFileName("");
    featureFileName << "match/segment_" << segCount << ".csv";
    featureFile.open(featureFileName.str().c_str());
    for(int feat_it=0; feat_it < histSize; feat_it++){
      featureFile << feature.at(feat_it) << ",";
    }
    segCount++;
    featureFile << std::endl;
    featureFile.close();
    usleep(2000);
        
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
  
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  
  ros::ServiceServer serv = n.advertiseService("/object_recognition", recognize_cb);
  
  ROS_INFO("Data Collection node ready.");
  
  ros::spin();

  
}

