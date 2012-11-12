//test for mantis_object_recognition node
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include <flann/flann.h>

#include <boost/filesystem.hpp>

#include "cph.h"
#include "mantis_perception/mantis_recognition.h"
#include "mantis_perception/mantis_segmentation.h"
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"
#include "tabletop_object_detector/TabletopSegmentation.h"

ros::ServiceClient recognition_client, segmentation_client;

ros::Publisher recognition_pub;
ros::Publisher seg_pub;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_recognition_test");
  ros::NodeHandle n;

  recognition_client = n.serviceClient<mantis_perception::mantis_recognition>("/mantis_object_recognition");
  segmentation_client = n.serviceClient<mantis_perception::mantis_segmentation>("/mantis_segmentation", true);

  seg_pub = n.advertise<sensor_msgs::PointCloud2>("/segmentation_result",1);

  while (ros::ok())
  {
    mantis_perception::mantis_segmentation seg_srv;
    if (!segmentation_client.call(seg_srv))
    {
      ROS_ERROR("Call to mantis segmentation service failed");
    }

    sensor_msgs::PointCloud2 segcluster;
    sensor_msgs::PointCloud clustervector;
    clustervector=seg_srv.response.clusters[0];
    sensor_msgs::convertPointCloudToPointCloud2(clustervector, segcluster);
    segcluster.header.frame_id=seg_srv.response.table.pose.header.frame_id;
    segcluster.header.stamp=seg_srv.response.table.pose.header.stamp;
    seg_pub.publish (segcluster);

    mantis_perception::mantis_recognition rec_srv;

    rec_srv.request.clusters = seg_srv.response.clusters;
    rec_srv.request.table = seg_srv.response.table;

    if (!recognition_client.call(rec_srv))
    {
      ROS_ERROR("Call to mantis recognition service failed");
    }

    ROS_INFO("Model label: %s", rec_srv.response.label.c_str());
    ROS_INFO("Model id: %d", rec_srv.response.model_id);
  
    ros::spinOnce();
  }//end ros ok while loop

}




