#include "ros/ros.h"
#include "mantis_perception/mantis_segmentation.h"

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

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>

//#include "nrg_object_recognition/segmentation.h"

  ros::Publisher plane_pub;
  ros::Publisher bound_pub;
  ros::Publisher cluster_pub;

class MantisSegmentor
{

  typedef pcl::PointXYZRGB    Point;

  private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher marker_pub_;
  //! Service server for object detection
  ros::ServiceServer segmentation_srv_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! Min number of inliers for reliable plane detection
  int inlier_threshold_;
  //! Size of downsampling grid before performing plane detection
  double plane_detection_voxel_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
  //! Filtering of original point cloud along the z, y, and x axes
  double z_filter_min_, z_filter_max_;
  double y_filter_min_, y_filter_max_;
  double x_filter_min_, x_filter_max_;
  //! Filtering of point cloud in table frame after table detection
  double table_z_filter_min_, table_z_filter_max_;
  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;
  //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;
  bool flatten_table_;
  //! How much the table gets padded in the horizontal direction
  double table_padding_;

  //! A tf transform listener
  tf::TransformListener listener_;
  //------------------ Callbacks -------------------

  //! Callback for service calls
  bool serviceCallback(mantis_perception::mantis_segmentation::Request &request,
                       mantis_perception::mantis_segmentation::Response &response);

  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  void processCloud(const sensor_msgs::PointCloud2 &cloud,
                      mantis_perception::mantis_segmentation::Response &seg_response,
                      tabletop_object_detector::Table table);

  public:

  MantisSegmentor(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    segmentation_srv_ = nh_.advertiseService(nh_.resolveName("segmentation_srv"),
                                             &MantisSegmentor::serviceCallback, this);

    //initialize operational flags
    priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
    priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
    priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
    priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
    priv_nh_.param<double>("z_filter_max", z_filter_max_, 1.25);
    priv_nh_.param<double>("y_filter_min", y_filter_min_, -1.0);
    priv_nh_.param<double>("y_filter_max", y_filter_max_, 1.0);
    priv_nh_.param<double>("x_filter_min", x_filter_min_, -1.0);
    priv_nh_.param<double>("x_filter_max", x_filter_max_, 1.0);
    priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.01);
    priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 0.50);
    priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
    priv_nh_.param<std::string>("processing_frame", processing_frame_, "");
    priv_nh_.param<double>("up_direction", up_direction_, -1.0);
    priv_nh_.param<bool>("flatten_table", flatten_table_, false);
    priv_nh_.param<double>("table_padding", table_padding_, 0.0);
    if(flatten_table_) ROS_DEBUG("flatten_table is true");
    else ROS_DEBUG("flatten_table is false");

  }

    //! Empty stub
    ~MantisSegmentor() {}
};
//! Callback for service calls
bool MantisSegmentor::serviceCallback(mantis_perception::mantis_segmentation::Request &request,
                     mantis_perception::mantis_segmentation::Response &response)
{
  ros::Time start_time = ros::Time::now();
  std::string topic = nh_.resolveName("cloud_in");
  ROS_INFO("Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());

  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));

  if (!recent_cloud)
  {
    ROS_ERROR("Tabletop object detector: no point_cloud2 has been received");
    response.result = response.NO_CLOUD_RECEIVED;
    return true;
  }

  //pcl::PointCloud<Point>::Ptr table_hull (new pcl::PointCloud<Point>);
  ROS_INFO_STREAM("Point cloud received after " << ros::Time::now() - start_time << " seconds; processing");
  if (!processing_frame_.empty())
  {
    //convert cloud to processing_frame_ (usually base_link)
    sensor_msgs::PointCloud old_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
    int current_try=0, max_tries = 3;
    while (1)
    {
      bool transform_success = true;
      try
      {
        listener_.transformPointCloud(processing_frame_, old_cloud, old_cloud);
      }
      catch (tf::TransformException ex)
      {
        transform_success = false;
        if (++current_try >= max_tries)
        {
          ROS_ERROR("Failed to transform cloud from frame %s into frame %s in %d attempt(s)", old_cloud.header.frame_id.c_str(),
                    processing_frame_.c_str(), current_try);
          response.result = response.OTHER_ERROR;
          return true;
        }
        ROS_DEBUG("Failed to transform point cloud, attempt %d out of %d, exception: %s", current_try, max_tries, ex.what());
        //sleep a bit to give the listener a chance to get a new transform
        ros::Duration(0.1).sleep();
      }
      if (transform_success) break;
    }
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
    ROS_INFO_STREAM("Input cloud converted to " << processing_frame_ << " frame after " <<
                    ros::Time::now() - start_time << " seconds");
    processCloud(converted_cloud, response, request.table);
    //clearOldMarkers(converted_cloud.header.frame_id);
  }
  else
  {
    processCloud(*recent_cloud, response, request.table);
    //clearOldMarkers(recent_cloud->header.frame_id);
  }

  //add the timestamp from the original cloud
  response.table.pose.header.stamp = recent_cloud->header.stamp;
  for(size_t i; i<response.clusters.size(); i++)
  {
    response.clusters[i].header.stamp = recent_cloud->header.stamp;
  }

  ROS_INFO_STREAM("In total, segmentation took " << ros::Time::now() - start_time << " seconds");
  return true;
}


//! Complete processing for new style point cloud
void MantisSegmentor::processCloud(const sensor_msgs::PointCloud2 &in_cloud,
                  mantis_perception::mantis_segmentation::Response &seg_response, tabletop_object_detector::Table table)
{
  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "segmenting image..." << std::endl;
  pcl::fromROSMsg(in_cloud, *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  //std::cout << "Voxel grid filtering...\n";
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_0 (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered_0);
  //std::cout << "done.\n";

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.0075);

  //std::cout << "Spatial filtering...\n";
  //Spatial filter.
  cloud_filtered->resize(0);
  //Parameters
 float min_x = x_filter_min_, max_x = x_filter_max_;
 float min_y = y_filter_min_, max_y = y_filter_max_;
 float min_z = z_filter_min_, max_z = z_filter_max_;

//  float min_x = -.2, max_x = .2;
//  float min_y = -.5, max_y = .5;
//  float min_z = .55, max_z = 1.15;

 for(pcl::PointCloud<pcl::PointXYZ>::iterator position=cloud_filtered_0->begin(); position!=cloud_filtered_0->end(); position++){
     //if(position->x > min_x && position->x < max_x && position->y > min_y && position->y < max_y && position->z > min_z && position->z < max_z)
     if(position->x > min_x && position->x < max_x && position->y > (2.145*position->z - 3.31) && position->y < (-.466*position->z + .400))
     cloud_filtered->push_back(*position);
  }
  sensor_msgs::PointCloud2 cloud_filtered_pc2;
  pcl::toROSMsg(*cloud_filtered, cloud_filtered_pc2);
  cloud_filtered_pc2.header = in_cloud.header;
  bound_pub.publish(cloud_filtered_pc2);

  //std::cout << "num points in spatially filtered cloud: " << cloud_filtered->points.size() << std::endl;
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    //std::cout << "extracting planar inliers..."  << std::endl;
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Publish dominant plane
    extract.filter (*cloud_plane);
    sensor_msgs::PointCloud2 plane_pc2;
    pcl::toROSMsg(*cloud_plane, plane_pc2);
    plane_pc2.header = in_cloud.header;
    plane_pub.publish(plane_pc2);

    //std::cout << "Removing tabletop..."  << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
    //std::cout << "done."  << std::endl;
  }

  pcl::toROSMsg(*cloud_filtered, cloud_filtered_pc2);
  cloud_filtered_pc2.header = in_cloud.header;
  cluster_pub.publish(cloud_filtered_pc2);

     //std::cout << "Number of points in remaining clusters: " << cloud_filtered->points.size()  << std::endl;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.03); // 2cm
  ec.setMinClusterSize (0);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  //std::cout << "length of cluster_indices: " << cluster_indices.size() << std::endl;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "writing cluster to service response. It has " << cloud_cluster->points.size() << " points.\n";
    sensor_msgs::PointCloud2 tempROSMsg;
    pcl::toROSMsg(*cloud_cluster, tempROSMsg);
    seg_response.clusters.push_back(tempROSMsg);
    j++;
  }

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "mantis_segmentation_node");
  ros::NodeHandle nh;

  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("/dominant_plane",1);
  bound_pub = nh.advertise<sensor_msgs::PointCloud2>("/bounded_scene",1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/pre_clustering",1);
  //ros::ServiceServer serv = n.advertiseService("/segmentation", segment_cb);

  MantisSegmentor node(nh);

  ros::spin();
  return 0;

}
