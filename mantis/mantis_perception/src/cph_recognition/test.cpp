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
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

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
ros::Publisher text_pub;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_recognition_test");
  ros::NodeHandle n;


  //get some parameters that will be global to the move base node
  ros::NodeHandle private_nh("~");
  std::string arm_name_;
  const std::string PARAM_ARM_NAME_DEFAULT= "arm_name";
  const std::string ARM_NAME_DEFAULT= "";
  private_nh.param(PARAM_ARM_NAME_DEFAULT, arm_name_, ARM_NAME_DEFAULT);


  recognition_client = n.serviceClient<mantis_perception::mantis_recognition>(arm_name_ + "/mantis_object_recognition");
  segmentation_client = n.serviceClient<mantis_perception::mantis_segmentation>(arm_name_ + "/tabletop_segmentation", true);

  text_pub = n.advertise<visualization_msgs::Marker>("/text_messages",1);

  tf::TransformListener listener;
  geometry_msgs::PointStamped object_point;

  while (ros::ok())
  {

    mantis_perception::mantis_segmentation seg_srv;
    if (!segmentation_client.call(seg_srv))
    {
      ROS_ERROR("Call to mantis segmentation service failed");
    }

    mantis_perception::mantis_recognition rec_srv;

    rec_srv.request.clusters = seg_srv.response.clusters;
    rec_srv.request.table = seg_srv.response.table;

    if (!recognition_client.call(rec_srv))
    {
      ROS_ERROR("Call to mantis recognition service failed");
    }

    std::size_t found;
    std::string label = rec_srv.response.label;
    found=label.find_last_of("/");
    ROS_INFO("Model label: %s", label.substr(found+1).c_str());
    ROS_INFO("Model id: %d", rec_srv.response.model_id);
    ROS_INFO_STREAM("Angle: " << rec_srv.response.pose.rotation );

    visualization_msgs::Marker marker;
    marker.header.frame_id = seg_srv.response.table.pose.header.frame_id;
    marker.header.stamp = seg_srv.response.table.pose.header.stamp;
    //marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text=label.substr(found+1).c_str();
    marker.pose.position.x = 0.5;
    marker.pose.position.y = 0;
    marker.pose.position.z = 1;
    //marker.pose.orientation.x = 0.0;
    //marker.pose.orientation.y = 0.0;
    //marker.pose.orientation.z = 0.0;
    //marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    text_pub.publish( marker );

/*
    object_point.header.frame_id = "/base_link";
    object_point.header.stamp = ros::Time();
    object_point.point.x = rec_srv.response.pose.x;
    object_point.point.y = rec_srv.response.pose.y;
    object_point.point.z = rec_srv.response.pose.z;
	  try{
		geometry_msgs::PointStamped base_point;
		listener.transformPoint("/object_training_frame", object_point, base_point);

		ROS_INFO("object_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
				object_point.point.x, object_point.point.y, object_point.point.z,
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
	  }
	  catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"pvct_training_frame\" to \"base_link\": %s", ex.what());
	  }
*/
    ros::spinOnce();
  }//end ros ok while loop

}




