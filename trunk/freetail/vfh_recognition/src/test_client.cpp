#include "ros/ros.h"
#include "vfh_recognition/Recognize.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <household_objects_database_msgs/DatabaseModelPoseList.h>

ros::ServiceClient client;
ros::Publisher pub;

void srv_cb(const sensor_msgs::PointCloud2 fromKinect)
{
  tabletop_object_detector::TabletopObjectRecognition rec_srv;
  if(client.call(rec_srv))
  {
   if(!rec_srv.response.models.empty()){
    pub.publish(rec_srv.response);
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
  
  pub = n.advertise<household_objects_database_msgs::DatabaseModelPoseList>("/recognition_result", 1);
  
  ros::spin();

  return 0;
}