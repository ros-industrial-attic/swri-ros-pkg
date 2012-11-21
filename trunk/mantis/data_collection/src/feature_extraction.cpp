#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/point_cloud.h>

#include "mantis_data_collection/process_cloud.h"
#include "euclidean_segmentation.h"
#include "nrg_object_recognition/segmentation.h"
#include "cph.h"

#include "tabletop_object_detector/Table.h"
#include "mantis_perception/mantis_segmentation.h"


ros::ServiceClient seg_client;
  
bool cloud_cb(mantis_data_collection::process_cloud::Request &req,
	      mantis_data_collection::process_cloud::Response &res)
{
  ROS_INFO("starting feature extraction process");
  //Segment from cloud:
  //May need to tweak segmentation parameters to get just the cluster I'm looking for.
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  /*nrg_object_recognition::segmentation seg_srv;

  seg_srv.request.scene = req.in_cloud;
  seg_srv.request.min_x = -.75, seg_srv.request.max_x = .4;
  seg_srv.request.min_y = -5, seg_srv.request.max_y = .5;
  seg_srv.request.min_z = 0.0, seg_srv.request.max_z = 1.15;
  seg_client.call(seg_srv);
  */
  //SegmentCloud(req.in_cloud, clouds);
  
  //For parameter tweaking, may be good to write all cluseters to file here for examination in pcd viewer.
  

  //cluster = clouds.at(0);
  //pcl::fromROSMsg(seg_srv.response.clusters.at(0), *cluster);
  //std::cout << "found " << clouds.size() << " clusters.\n"; 

    mantis_perception::mantis_segmentation seg_srv;
    seg_client.call(seg_srv);
    ROS_INFO("mantis segmentation called");

    if (!seg_client.call(seg_srv))
           {
              ROS_ERROR("Call to mantis segmentation service failed");

            }
    sensor_msgs::PointCloud2 segcluster;
    sensor_msgs::PointCloud clustervector;
    clustervector=seg_srv.response.clusters[0];
    sensor_msgs::convertPointCloudToPointCloud2(clustervector, segcluster);
    segcluster.header.frame_id="/base_link";
    //segcluster.header.stamp=seg_srv.response.table.pose.header.stamp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(segcluster,*cluster);

  std::cout << "cluster has " << cluster->height*cluster->width << " points.\n";
  
  //Write raw pcd file (objecName_angle.pcd)
  std::stringstream fileName_ss;
  fileName_ss << "data/" << req.objectName << "_" << req.angle << ".pcd";
  std::cout << "writing raw cloud to file...\n";
  std::cout << fileName_ss.str() << std::endl;
  pcl::io::savePCDFile(fileName_ss.str(), *cluster);
  std::cout << "done.\n";
 
  //Write vfh feature to file: 
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cluster);
  //Estimate normals:
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cluster);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
  vfh.setInputNormals (cloud_normals);
  //Estimate vfh:
  vfh.setSearchMethod (tree);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  // Compute the feature
  vfh.compute (*vfhs);
  //Write to file: (objectName_angle_vfh.pcd)
  fileName_ss.str("");
  std::cout << "writing vfh descriptor to file...\n";
  fileName_ss << "data/" << req.objectName << "_" << req.angle << "_vfh.pcd";
  pcl::io::savePCDFile(fileName_ss.str(), *vfhs);
  std::cout << "done.\n";

 
  //Extract cph
  std::vector<float> feature;
  CPHEstimation cph(5,72);
  cph.setInputCloud(cluster);
  cph.compute(feature);
  //Write cph to file. (objectName_angle.csv)
  std::ofstream outFile;
  fileName_ss.str("");
  fileName_ss << "data/" << req.objectName << "_" << req.angle << ".csv";
  outFile.open(fileName_ss.str().c_str());
  std::cout << "writing cph descriptor to file...\n";
  for(unsigned int j=0; j<feature.size(); j++){
	outFile << feature.at(j) << " "; 
  }
  outFile.close();
  fileName_ss.str("");
  std::cout << "done.\n";
  res.result = 1;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "feature_extractor");
  
  ros::NodeHandle n;
  
  
  //Offer services that can be called from the terminal:
  ros::ServiceServer joint_test_serv = n.advertiseService("process_cloud", cloud_cb );
  //seg_client = n.serviceClient<nrg_object_recognition::segmentation>("segmentation");
  seg_client = n.serviceClient<mantis_perception::mantis_segmentation>("mantis_segmentation", true);
  //Set up service clients:
 
  ros::spin();

  


  return 0;
}
