#include "ros/ros.h"
#include <algorithm>

#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/PointCloud2.h"
#include "freetail_ball_bin_segmentation/clusterclouds.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "ros/console.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopSegmentation.h"
#include </opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopObjectRecognition.h>
/**
 * This node will subscribe to point cloud data from the Kinect and perform a table top segmentation
 */

ros::Publisher pub;
ros::ServiceClient seg_srv;
ros::ServiceClient rec_srv;

//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void pointcloudcallback(const sensor_msgs::PointCloud2ConstPtr& pcmsg)
{
  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*pcmsg, *in_cloud);
	//sensor_msgs::PointCloud2::Ptr cloud_filtered;
	//pcl::PassThrough<sensor_msgs::PointCloud2> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // Perform the actual filtering - First cut out far stuff
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(in_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.3);
  pass.filter(*cloud_filtered);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  //pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        ROS_INFO("Could not estimate a planar model for the given dataset.");
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Write the planar inliers to disk
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered = cloud_f;
    }
  // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            //push_back: add a point to the end of the existing vector
                    cloud_cluster->points.push_back(in_cloud->points[*pit]);
            }

            //Merge current clusters to whole point cloud
        *clustered_cloud += *cloud_cluster;

      }

    pcl::toROSMsg (*clustered_cloud , *clusters);
    //clusters.frame_id="kinect_ir_optical_frame";
    pub.publish (*clusters);
    /*
  // Publish the data
  pub.publish (*cloud_filtered);

*/

}
void segment(tabletop_object_detector::TabletopSegmentation& segmentation_srv) {
	//tabletop_object_detector::TabletopSegmentation segmentation_srv;
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

}

void publishSegmentedPoints(std::vector<sensor_msgs::PointCloud> clustervector) //freetail_ball_bin_segmentation::clustercloudsPtr&
{
	//std::vector<sensor_msgs::PointCloud2> pclVector;
	//pclVector=clusters->pointClouds;

/*  sensor_msgs::PointCloud segmented;
  sensor_msgs::PointCloud pc;
  PointCloud::Ptr msg (new PointCloud);
  for (unsigned int i =0; i < clusters.size(); i++)
  {
	  pc = clusters[i];
	  msg->header = pc.header;
	  //msg->channels = pc.channels;
	  msg->points = pc.points;

  }
  //std::vector<sensor_msgs::PointCloud2> segmented;
  //freetail_ball_bin_segmentation::clusterclouds segmented;
  //segmented = clusters->pointClouds;
 // sensor_msgs::Image segmented;
  //pcl::toROSMsg(clusters, segmented);*/

  //pub.publish(segmented);
}

int main(int argc, char **argv)
{

//* You must call one of the versions of ros::init() before using any other part of the ROS system.*

  ros::init(argc, argv, "bin_filter");

  ros::NodeHandle n;

  //subscribe to Kinect point xyzrgb point cloud data
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, pointcloudcallback);

  //create the segmentation and recognition service clients
  seg_srv = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);
/*  //rec_srv = n.serviceClient<tabletop_object_detector::TabletopObjectRecognition>
  ("/tabletop_object_recognition", true);
  //tabletop_object_detector::TabletopSegmentation segmentation;
  //segment(segmentation);

  //publishSegmentedPoints(segmentation.response.clusters);
*/

  //publish topic for rviz visualization
  pub = n.advertise<sensor_msgs::PointCloud2>("clusters", 100);

  ros::spin();

  return EXIT_SUCCESS;
}
