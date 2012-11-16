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

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <nrg_object_recognition/recognition.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
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
int num_ybins = 5; int num_rbins = 72;
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


bool recognize_cb(nrg_object_recognition::recognition::Request &srv_request,
		  nrg_object_recognition::recognition::Response &srv_response)
{
  
  //create knn index
  flann::Index<flann::ChiSquareDistance<float> > index (*data, flann::LinearIndexParams ());
  index.buildIndex ();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(srv_request.cluster, *cluster);
  
  // Create cph estimation.
  CPHEstimation cph(num_ybins,num_rbins);
    
  //Hold results:
  std::string label, angleStr;
  int angle;
  Eigen::Vector4f translation;
    
  //Demean the cloud.
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cluster, centroid);
  srv_response.pose.x = centroid(0);
  srv_response.pose.y = centroid(1);
  srv_response.pose.z = centroid(2);

  //Compute cph:
  cph.setInputCloud (cluster);
  std::vector<float> feature;
  //std::cout << "computing feature... ";
  cph.compute(feature);
  //std::cout << "done.\n";

  //Algorithm parameters  
 // float thresh = 280; //similarity threshold
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
  if(k_distances[0][0] < srv_request.threshold){
    //std::cout << "distance: " << k_distances[0][0] << std::endl;
    //ROS_INFO("%f", k_distances[0][0]);
    
    //Load nearest match
    std::string cloud_name = models.at(k_indices[0][0]).first;

    cloud_name.erase(cloud_name.end()-3, cloud_name.end());
    angleStr.assign(cloud_name.begin()+cloud_name.rfind("_")+1, cloud_name.end());
    std::string label;
    label.assign(cloud_name.begin()+5, cloud_name.begin()+cloud_name.rfind("_"));
    angle = atoi(angleStr.c_str());
    srv_response.label = label;
    srv_response.pose.rotation = angle;        
  }
  return(1);
}


int main(int argc, char **argv)
{  
  
  ros::init(argc, argv, "cph_recongition_node");
  ros::NodeHandle n;
  
  loadFeatureModels (argv[1], ".csv", models);
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data\n", 
      (int)models.size ());

  // Convert data into FLANN format
  data = new flann::Matrix<float> (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
  
  std::cout << "data size: [" << data->rows << " , " << data->cols << "]\n";
  for (size_t i = 0; i < data->rows; ++i)
    for (size_t j = 0; j < data->cols; ++j)
      *(data->ptr()+(i*data->cols + j)) = models[i].second[j];
    
  pcl::console::print_error ("Training data loaded.\n");
  
  ros::ServiceServer serv = n.advertiseService("/cph_recognition", recognize_cb);
  
  ROS_INFO("cph_recognition_node ready.");
  
  ros::spin(); 

  return(1);
}

