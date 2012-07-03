#include "ros/ros.h"
#include "vfh_recognition/Recognize.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "euclidean_segmentation_rgba.h"
//#include "vfh_recognition::Recognize_msg.h"


ros::ServiceClient client;
ros::Publisher pub;

void srv_cb(const sensor_msgs::PointCloud2 fromKinect)
{
   vfh_recognition::Recognize rec_srv;
   rec_srv.request.scene = fromKinect;
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
   pcl::fromROSMsg(fromKinect, *cloud);
   pcl::io::savePCDFileBinary("scene.pcd", *cloud);
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> segments;
   SegmentCloud(cloud, segments);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  
  ros::NodeHandle n;
  
  //Set up Kinect subscriber
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, srv_cb);
  
  ros::spin();

  return 0;
}