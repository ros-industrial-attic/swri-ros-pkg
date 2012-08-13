#include "ros/ros.h"
#include <algorithm>
using std::max;
using std::min;

#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "ros/console.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
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
 * This node will subscribe to point cloud data from the Kinect and perform segment objects down to ping pong balls
 */

ros::Publisher pub;
ros::ServiceClient seg_srv;
ros::ServiceClient rec_srv;

//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void pointcloudcallback(const sensor_msgs::PointCloud2ConstPtr& pcmsg)
{
  sensor_msgs::PointCloud2 clusters;//

  //CALL TABLETOP SEGMENTATION SERVICE WHICH WILL OUTPUT A VECTOR OF CLUSTERS (AND A TABLE)
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

      ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());

  //get the first and largest cluster and convert it to a PointCloud2 for publishing (making sure to include a
  //frame id and a stamp)
  sensor_msgs::PointCloud clustervector;
  clustervector=segmentation_srv.response.clusters[0];
  sensor_msgs::convertPointCloudToPointCloud2(clustervector, clusters);
  clusters.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  clusters.header.stamp=segmentation_srv.response.table.pose.header.stamp;

  //publish clusters for rviz
  pub.publish (clusters);

  //TAKE CLUSTER OF BIN/BALL AND REMOVE RED (BIN)

  /* SEGMENTATION USING ONLY PCL ALGORITHMS

  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*pcmsg, *in_cloud);
	//sensor_msgs::PointCloud2::Ptr cloud_filtered;
	//pcl::PassThrough<sensor_msgs::PointCloud2> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // Perform the actual filtering - Cut out far stuff
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(in_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.1, 1.5);
  pass.filter(*cloud_filtered);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);

  int nr_points = (int) cloud_filtered->points.size ();
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
      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " "
                                            << coefficients->values[3] << std::endl;


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
    ec.setClusterTolerance (0.01); // 2cm
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
             cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
             }
        //Merge current clusters to whole point cloud
        *clustered_cloud += *cloud_cluster;
    }

    pcl::toROSMsg (*clustered_cloud, clusters);
    clusters.header.frame_id="/kinect_ir_optical_frame";
    clusters.header.stamp=ros::Time::now();


  // Publish the data
    pub.publish (clusters);

 */
}


/*void segment(tabletop_object_detector::TabletopSegmentation& segmentation_srv) {
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

}*/


int main(int argc, char **argv)
{

//* You must call one of the versions of ros::init() before using any other part of the ROS system.*

  ros::init(argc, argv, "bin_filter");

  ros::NodeHandle n;

  //subscribe to Kinect point xyzrgb point cloud data
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, pointcloudcallback);

  //create the segmentation service clients
  seg_srv = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);

/*  //tabletop_object_detector::TabletopSegmentation segmentation;
  //segment(segmentation);
*/

  //publish topic for rviz visualization
  pub = n.advertise<sensor_msgs::PointCloud2>("clusters", 100);

  ros::spin();

  return EXIT_SUCCESS;
}
