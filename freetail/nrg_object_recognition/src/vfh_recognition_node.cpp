//cph_recognition_node.cpp
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <fstream>

#include <flann/flann.h>
//#include <flann/io/hdf5.h>

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <nrg_object_recognition/recognition.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"


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

bool recognize_cb(nrg_object_recognition::recognition::Request &srv_request,
		  nrg_object_recognition::recognition::Response &srv_response)
{
  
  //create knn index
  flann::Index<flann::ChiSquareDistance<float> > index (*data, flann::LinearIndexParams ());
  index.buildIndex ();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(srv_request.cluster, *cluster);
  
  // Create vfh estimation.
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    
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
  
  vfh.setInputCloud (cluster);
  //Estimate normals:
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cluster);
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
  //float thresh = 280; //similarity threshold
  int k = 1; //number of neighbors
  //KNN classification
  nearestKSearch (index, histogram, k, k_indices, k_distances);
  
  //determine label and pose:
  if(k_distances[0][0] < srv_request.threshold){
    //std::cout << "distance: " << k_distances[0][0] << std::endl;
    //ROS_INFO("%f", k_distances[0][0]);
    
    //Load nearest match
    std::string cloud_name = models.at(k_indices[0][0]).first;

    cloud_name.erase(cloud_name.end()-8, cloud_name.end()-4);
    std::string recognitionLabel, viewNumber;
    recognitionLabel.assign(cloud_name.begin()+5, cloud_name.begin()+cloud_name.rfind("_"));
    viewNumber.assign(cloud_name.begin()+cloud_name.rfind("_")+1, cloud_name.end()-4);
    srv_response.label = recognitionLabel;
    //std::cout << "pose: " << viewNumber << std::endl;
    srv_response.pose.rotation = atof(viewNumber.c_str());        
  }
  return(1);
}


int main(int argc, char **argv)
{  
  
  ros::init(argc, argv, "vfh_recongition_node");
  ros::NodeHandle n;
  
  loadFeatureModels (argv[1], ".pcd", models);
  pcl::console::print_highlight ("Loaded %d VFH models. Creating training data\n", 
      (int)models.size ());

  // Convert data into FLANN format
  data = new flann::Matrix<float> (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
  
  std::cout << "data size: [" << data->rows << " , " << data->cols << "]\n";
  for (size_t i = 0; i < data->rows; ++i)
    for (size_t j = 0; j < data->cols; ++j)
      *(data->ptr()+(i*data->cols + j)) = models[i].second[j];
    
  pcl::console::print_error ("Training data loaded.\n");
  
  ros::ServiceServer serv = n.advertiseService("/vfh_recognition", recognize_cb);
  
  ROS_INFO("vfh_recognition_node ready.");
  

  ros::spin(); 

  return(1);
}

