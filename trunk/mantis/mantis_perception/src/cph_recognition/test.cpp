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

//float subtract_angle(float angle_1, float angle_2);

ros::ServiceClient recognition_client, seg_client;



//ros::Publisher pan_pub;
ros::Publisher recognition_pub;



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_object_recognition");
  ros::NodeHandle n;

  //ros::ServiceServer rec_serv = n.advertiseService("/recognition_service", rec_cb);
  recognition_client = n.serviceClient<nrg_object_recognition::recognition>("mantis_object_recognition");
  seg_client = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);
  //ros::Subscriber kin_sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
  //pan_pub = n.advertise<std_msgs::UInt16>("/pan_command",1);
  
  tabletop_object_detector::TabletopSegmentation seg_srv;
        if (!seg_client.call(seg_srv))
           {
              ROS_ERROR("Call to table top segmentation service failed");

            }

    ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)seg_srv.response.clusters.size());
  

  
  mantis_perception::mantis_recognition rec_srv;
    
  if (!recognition_client.call(rec_srv))
             {
                ROS_ERROR("Call to mantis recognition service failed");

              }

  rec_srv.request.clusters = seg_srv.response.clusters;

  
  ROS_INFO("Model label: %s", rec_srv.response.label.c_str());
  ROS_INFO("Model label: %d", rec_srv.response.model_id);
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
  
  ros::spin(); 
}




