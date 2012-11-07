//main_dataset_node
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
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"
#include "tabletop_object_detector/TabletopSegmentation.h"
#include "nrg_object_recognition/segmentation.h"


ros::ServiceClient recognition_client, segmentation_client;

ros::Publisher recognition_pub;
ros::Publisher seg_pub;
/*sensor_msgs::PointCloud2 cloud_to_process;
void kinect_cb(sensor_msgs::PointCloud2 fromKinect)
{
cloud_to_process = fromKinect;
}*/
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_recognition_test");
  ros::NodeHandle n;

  //ros::ServiceServer rec_serv = n.advertiseService("/recognition_service", rec_cb);
  recognition_client = n.serviceClient<mantis_perception::mantis_recognition>("/mantis_object_recognition");
  segmentation_client = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);
  //segmentation_client = n.serviceClient<nrg_object_recognition::segmentation>("segmentation");
  //ros::Subscriber kin_sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  seg_pub = n.advertise<sensor_msgs::PointCloud2>("/segmentation_result",1);

  while (ros::ok())
  {
    tabletop_object_detector::TabletopSegmentation seg_srv;
        if (!segmentation_client.call(seg_srv))
           {
              ROS_ERROR("Call to tabletop segmentation service failed");

            }
	sensor_msgs::PointCloud2 segcluster;
    sensor_msgs::PointCloud clustervector;
    clustervector=seg_srv.response.clusters[0];
    sensor_msgs::convertPointCloudToPointCloud2(clustervector, segcluster);
    segcluster.header.frame_id=seg_srv.response.table.pose.header.frame_id;
    segcluster.header.stamp=seg_srv.response.table.pose.header.stamp;
    seg_pub.publish (segcluster);

    /*nrg_object_recognition::segmentation seg_srv;

	seg_srv.request.scene = cloud_to_process;
    seg_srv.request.min_x = -.75, seg_srv.request.max_x = .5;
	seg_srv.request.min_y = -.25, seg_srv.request.max_y = 1.0;
	seg_srv.request.min_z = 0.0, seg_srv.request.max_z = 1.3;
	segmentation_client.call(seg_srv);
	if (!segmentation_client.call(seg_srv))
	{
	  ROS_ERROR("Call to nrg segmentation service failed");
	}

    ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)seg_srv.response.clusters.size());

    seg_pub.publish (seg_srv.response.clusters.at(0));*/
  
    mantis_perception::mantis_recognition rec_srv;
    rec_srv.request.clusters = seg_srv.response.clusters;
    rec_srv.request.table = seg_srv.response.table;

    if (!recognition_client.call(rec_srv))
	  {
        ROS_ERROR("Call to mantis recognition service failed");
	  }

    ROS_INFO("Model label: %s", rec_srv.response.label.c_str());
    ROS_INFO("Model id: %d", rec_srv.response.model_id);
/*
  //Visualization://////////////////////////////////////////////////////
        //build filename.
        std::stringstream fileName;
        fileName << "data/" << rec_srv.response.label << "_" << rec_srv.response.pose.rotation << ".pcd";
        //Load and convert file.
        pcl::PointCloud<pcl::PointXYZ>::Ptr trainingMatch (new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 rosMsg;
        pcl::io::loadPCDFile(fileName.str(), *trainingMatch);
        //Translate to location:
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*trainingMatch, centroid);
        pcl::demeanPointCloud<pcl::PointXYZ> (*trainingMatch, centroid, *trainingMatch);

        Eigen::Vector3f translate;
        Eigen::Quaternionf rotate;
        translate(0) = rec_srv.response.pose.x;
        translate(1) = rec_srv.response.pose.y;
        translate(2) = rec_srv.response.pose.z;
        rotate.setIdentity();
        pcl::transformPointCloud(*trainingMatch, *trainingMatch, translate, rotate);

        pcl::toROSMsg(*trainingMatch, rosMsg);
        //Add transform to header
        rosMsg.header.frame_id = "/camera_depth_optical_frame";
        //Publish to topic /recognition_result.
        recognition_pub.publish(rosMsg);
        /////////end visualization////////////////////////////////////////////////////

  
  recognition_pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_test",1);
*/
  
    ros::spinOnce();
  }//end ros ok while look

}




