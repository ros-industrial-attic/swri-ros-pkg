#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <household_objects_database_msgs/DatabaseModelPoseList.h>
#include <Eigen/Core>
#include <pcl/registration/ia_ransac.h>

ros::ServiceClient client;
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 rosMsg;

void srv_cb(const sensor_msgs::PointCloud2 fromKinect)
{
  tabletop_object_detector::TabletopObjectRecognition rec_srv;
  if(client.call(rec_srv))
  {
   if(!rec_srv.response.models.empty()){
    pub.publish(rec_srv.response);
    std::stringstream fileName("");
    //Load the model to which the first recognition result refers:
    fileName << "data/segment_" << rec_srv.response.models[0].model_list[0].model_id << ".pcd";
    pcl::io::loadPCDFile(fileName.str().c_str(), *cloud);
    //Transform it using the returned transform:
    Eigen::Vector3f translate;
    Eigen::Quaternionf rotate;
    translate(0) = rec_srv.response.models[0].model_list[0].pose.pose.position.x;
    translate(1) = rec_srv.response.models[0].model_list[0].pose.pose.position.y;
    translate(2) = rec_srv.response.models[0].model_list[0].pose.pose.position.z;
    
    rotate.x() = rec_srv.response.models[0].model_list[0].pose.pose.orientation.x;
    rotate.y() = rec_srv.response.models[0].model_list[0].pose.pose.orientation.y;
    rotate.z() = rec_srv.response.models[0].model_list[0].pose.pose.orientation.z;
    rotate.w() = rec_srv.response.models[0].model_list[0].pose.pose.orientation.w;

    //rotate.setIdentity();
    pcl::transformPointCloud(*cloud, *cloud, translate, rotate);
    pcl::toROSMsg(*cloud, rosMsg);
    rosMsg.header.frame_id = "/camera_depth_optical_frame";
    pub.publish(rosMsg);
   }
     
  }else{
    ROS_ERROR("Failed to call object reognition service");
    return;
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, srv_cb);
  
  tabletop_object_detector::TabletopObjectRecognition rec_srv;  
  client = n.serviceClient<tabletop_object_detector::TabletopObjectRecognition>("/object_recognition");
  
  pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_result", 1);
  
  ros::spin();

  return 0;
}