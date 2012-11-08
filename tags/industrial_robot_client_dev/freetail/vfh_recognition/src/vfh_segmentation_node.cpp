/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
//vfh_segmentation_node.cpp

#define BOOST_FILESYSTEM_VERSION 2

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/ModelCoefficients.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <vfh_recognition/SupportClasses.h>

// global variables
sensor_msgs::PointCloud2 fromKinect;
RosParametersList ROS_PARAMS = RosParametersList();


bool segment_cb(tabletop_object_detector::TabletopSegmentation::Request &srv_request,
		  tabletop_object_detector::TabletopSegmentation::Response &srv_response)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::PointCloud<pcl::PointXYZ>::Ptr table;

  SegmentCloud(fromKinect, clusters, table, ROS_PARAMS);
  //Need to modify above function to return table.
  
//   Eigen::Vector4f centroid;
//   std::cout << "computing table centroid...";
//   pcl::compute3DCentroid (*table, centroid);
//   std::cout << " done.\n";
//   
//   std::cout << "copying table points... ";
//   float x_max=0, x_min=100, y_max=0, y_min = 100, z_max=0, z_min=100;
//   for(unsigned int i=0; i<table->size(); i++){
//     if(table->points.at(i).x > x_max)
//       x_max = table->points.at(i).x;
//     if(table->points.at(i).x < x_min)
//       x_min = table->points.at(i).x;
//     if(table->points.at(i).y > y_max)
//       y_max = table->points.at(i).y;
//     if(table->points.at(i).y < y_min)
//       y_min = table->points.at(i).y;
//     if(table->points.at(i).z > z_max)
//       z_max = table->points.at(i).z;
//     if(table->points.at(i).z < z_min)
//       z_min = table->points.at(i).z;
//   }
//   std::cout << " done.\n";

  //Build service response:
  //Table:
//   srv_response.table.pose.header.stamp = fromKinect.header.stamp;
//   srv_response.table.pose.header.frame_id = "/camera_depth_optical_frame"; //This will be loaded from the parameter server
//   srv_response.table.pose.pose.position.x = centroid(0); 
//   srv_response.table.pose.pose.position.y = centroid(1);
//   srv_response.table.pose.pose.position.z = centroid(2);
//   srv_response.table.pose.pose.orientation.x = 0;
//   srv_response.table.pose.pose.orientation.z = 0;
//   srv_response.table.pose.pose.orientation.y = 0;
//   srv_response.table.pose.pose.orientation.w = 1;
//   srv_response.table.x_min = x_min;
//   srv_response.table.x_max = x_max;
//   srv_response.table.y_min = y_min;
//   srv_response.table.y_max = y_max;
  //clusters:
  srv_response.clusters.resize(clusters.size());
  for(unsigned int i=0; i< clusters.size(); i++){
   srv_response.clusters.at(i).header.frame_id = "/camera_depth_optical_frame"; //This will be loaded from the parameter server
   srv_response.clusters.at(i).header.stamp = fromKinect.header.stamp;
   srv_response.clusters.at(i).points.resize(clusters.at(i)->points.size());
   for(unsigned int j=0; j< clusters.at(i)->points.size(); j++){ 
      srv_response.clusters.at(i).points.at(j).x = clusters.at(i)->points.at(j).x;
      srv_response.clusters.at(i).points.at(j).y = clusters.at(i)->points.at(j).y;
      srv_response.clusters.at(i).points.at(j).z = clusters.at(i)->points.at(j).z;
   }
   //channels not populated.
  }
 
  return(1);
}

void kinect_cb(const sensor_msgs::PointCloud2 inCloud)
{
  fromKinect = inCloud;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle n;

  // fetching values from ros parameter server
  ROS_PARAMS.loadParams(n,true);

  //ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  ros::Subscriber sub = n.subscribe(ROS_PARAMS.Vals.InputCloudTopicName, 1, kinect_cb);

  //ros::ServiceServer serv = n.advertiseService("/object_recognition", recognize_cb);
  ros::ServiceServer serv = n.advertiseService(ROS_PARAMS.Vals.SegmentationServiceName, segment_cb);
  ROS_INFO("Segmentation node ready.");
  ros::spin();


}
