#include "ros/ros.h"
#include <algorithm>

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "ros/console.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopSegmentation.h"
#include </opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopObjectRecognition.h>
/**
 * This node will subscribe to point cloud data from the Kinect and perform a table top segmentation
 */

ros::Publisher pub;
ros::ServiceClient seg_srv;
ros::ServiceClient rec_srv;


void pointcloudcallback(const sensor_msgs::PointCloud2ConstPtr& pcmsg)
{
  //pcl::PointCloud<pcl::PointXYZRGB> in_cloud;
/*  printf ("Cloud: width = %d, height = %d\n", pcmsg->width, pcmsg->height);
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, pcmsg->points)
    printf ("\t(%f, %f, %f, %u, %u, %u)\n", pt.x, pt.y, pt.z, pt.r, pt.g, pt.b);
*/
/*  sensor_msgs::PointCloud2 output;
  // Perform the actual filtering
  //First cut out far stuff
  pcl::PassThrough<sensor_msgs::PointCloud2> pass;
  pass.setInputCloud(pcmsg);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.3);
  pass.filter(output);
  //OR use a voxel grid
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud(pcmsg);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter (output);

  // Publish the data
  pub.publish (output);
*/
}
void segment() {
	tabletop_object_detector::TabletopSegmentation segmentation_srv;
    if (!seg_srv.call(segmentation_srv))
       {
          ROS_ERROR("Call to segmentation service failed");
          //return false;
        }
    if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
        {
          ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
          //return false;
        }

    //addDetectedTableToPlanningSceneDiff(segmentation_srv.response.table);
    ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
//   std::vector<sensor_msgs::PointCloud> output;
//    pub.publish (segmentation_srv.response.clusters);

}

void publishSegmentedPoints(std::vector<sensor_msgs::PointCloud2> clusters)
{
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::PointCloud2 final;

  final.height = 1;
  for(unsigned int i =0; i < clusters.size(); i++)
  {
    pc2 = clusters[i];
    final.header = pc2.header;
    final.fields = pc2.fields;
    final.width += pc2.width;
    final.is_bigendian = pc2.is_bigendian;
    final.point_step = pc2.point_step;
    final.row_step += pc2.row_step;
    final.data.insert(final.data.end(),pc2.data.begin(),pc2.data.end());
  }
  pub.publish(final);
}

int main(int argc, char **argv)
{

//* You must call one of the versions of ros::init() before using any other part of the ROS system.*

  ros::init(argc, argv, "bin_filter");

  ros::NodeHandle n;

  //subscribe to Kinect point xyzrgb point cloud data
  //ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, pointcloudcallback);

  //create the segmentation and recognition service clients
  seg_srv = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);
  //rec_srv = n.serviceClient<tabletop_object_detector::TabletopObjectRecognition>("/tabletop_object_recognition", true);

  segment();

//  publishSegmentedPoints(segmentation.response.clusters);
  //publish topic for rviz visualization
  pub = n.advertise<sensor_msgs::PointCloud>("clusters", 100);

  ros::spin();

  return EXIT_SUCCESS;
}
