#include "ros/ros.h"
#include "vfh_recognition/Recognize.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include "vfh_recognition::Recognize_msg.h"


ros::ServiceClient client;
ros::Publisher pub;

void srv_cb(const sensor_msgs::PointCloud2 fromKinect)
{
  vfh_recognition::Recognize rec_srv;
  //rec_srv.request.scene = fromKinect;
  
  //Sandbox:
  pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(fromKinect, *rawCloud);
  //Filter cloud to workspace:
   pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  for(unsigned int i = 0; i< rawCloud->size(); i++){
    if(rawCloud->points[i].x > -.7 && rawCloud->points[i].x < .7 && rawCloud->points[i].z < 2.1)
      filteredCloud->points.push_back(rawCloud->points[i]);
  }
  //std::cout << "Raw cloud size: " << rawCloud->size() << std::endl;
  //std::cout << "Filtered cloud size: " << filteredCloud->size() << std::endl;
  pcl::toROSMsg(*filteredCloud, rec_srv.request.scene);
   
  if (client.call(rec_srv))
  {
    sensor_msgs::PointCloud2 rec_model = rec_srv.response.model;
    rec_model.header.frame_id = "/camera_depth_optical_frame";
    pub.publish(rec_model);
  }
  else
  {
    ROS_ERROR("Failed to call object reognition service");
    return;
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  
  ros::NodeHandle n;
  
  //Set up Kinect subscriber
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, srv_cb);
  
  //Set up service client to request recongition from the recognition node
  client = n.serviceClient<vfh_recognition::Recognize>("/object_recognition");
  
  //Set up publisher to publish result.
  pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_result", 1);
  
  ros::spin();

  return 0;
}