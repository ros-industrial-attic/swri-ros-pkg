#include "ros/ros.h"
#include "mantis_perception/mantis_segmentation.h"
#include "tabletop_object_detector/TabletopSegmentation.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include "tabletop_object_detector/marker_generator.h"

  ros::Publisher plane_pub;
  ros::Publisher bound_pub;
  ros::Publisher cluster_pub;
  ros::Publisher first_cluster_pub;

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
  bool serviceCallback(tabletop_object_detector::TabletopSegmentation::Request &request,
		  tabletop_object_detector::TabletopSegmentation::Response &response);

  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  void processCloud(const sensor_msgs::PointCloud2 &cloud,
		  tabletop_object_detector::TabletopSegmentationResponse &seg_response,
                      tabletop_object_detector::Table table);

  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);

  tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs,
  		double up_direction, bool flatten_plane);

  template <typename PointT>
  bool getPlanePoints (const pcl::PointCloud<PointT> &table,
  		     const tf::Transform& table_plane_trans,
  		     sensor_msgs::PointCloud &table_points);

  template <class PointCloudType>
  tabletop_object_detector::Table getTable(std_msgs::Header cloud_header,
                                    const tf::Transform &table_plane_trans,
                                    const PointCloudType &table_points);

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
bool MantisSegmentor::serviceCallback(tabletop_object_detector::TabletopSegmentation::Request &request,
		tabletop_object_detector::TabletopSegmentation::Response &response)
{

/*
  static tf::TransformBroadcaster broadcaster;

  broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.545, -0.18, 0.049)),
			ros::Time::now(),"/base_link", "/object_training_frame"));
*/
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
    clearOldMarkers(converted_cloud.header.frame_id);
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

void MantisSegmentor::processCloud(const sensor_msgs::PointCloud2 &in_cloud,
		tabletop_object_detector::TabletopSegmentation::Response &seg_response, tabletop_object_detector::Table table)
{
  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "segmenting image..." << std::endl;
  pcl::fromROSMsg(in_cloud, *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  //std::cout << "Voxel grid filtering...\n";
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_0 (new pcl::PointCloud<pcl::PointXYZ>);
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

 for(pcl::PointCloud<pcl::PointXYZ>::iterator position=cloud_filtered_0->begin(); position!=cloud_filtered_0->end(); position++)
 //for (pcl::PointCloud<pcl::PointXYZ>::iterator position=cloud->begin(); position!=cloud->end(); position++)
 {
 if(position->x > min_x && position->x < max_x && position->y > min_y && position->y < max_y && position->z > min_z && position->z < max_z)
    // if(position->x > min_x && position->x < max_x && position->y > (2.145*position->z - 3.31) && position->y < (-.466*position->z + .400))
     cloud_filtered->push_back(*position);
  }

  sensor_msgs::PointCloud2 cloud_filtered_pc2;
  pcl::toROSMsg(*cloud_filtered, cloud_filtered_pc2);
  cloud_filtered_pc2.header = in_cloud.header;
  bound_pub.publish(cloud_filtered_pc2);

  //std::cout << "num points in spatially filtered cloud: " << cloud_filtered->points.size() << std::endl;
  int nr_points = (int) cloud_filtered->points.size ();
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
    extract.filter (*cloud_plane); //cloud_plane is a pcl::PointCloud<pcl::PointXYZ>::Ptr
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
  ec.setClusterTolerance (0.01); // 2cm
  ec.setMinClusterSize (min_cluster_size_);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::vector<sensor_msgs::PointCloud2> pc2_clusters;
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
    pc2_clusters.push_back(tempROSMsg);
    //seg_response.clusters.push_back(tempROSMsg);
    j++;
  }

//convert array of PointCloud2 to PointCloud
  //Filter high points and noisy points while doing so
  std::vector<sensor_msgs::PointCloud> out_clusters;
  ROS_INFO("Cluster published and put into PointCloud2 array");
  for (int i=0; i<pc2_clusters.size(); i++)
  {
	  sensor_msgs::PointCloud out_cloud;
	  sensor_msgs::PointCloud2 ocloud;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ> cloud_noise;
	  pcl::fromROSMsg (pc2_clusters.at(i), *cluster_ptr);

	  pcl::PassThrough<pcl::PointXYZ> pass;
	  pass.setInputCloud (cluster_ptr);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (0, 0.08);
	  pass.filter (*cloud_cut);

	  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> out_remove;
	  out_remove.setInputCloud(cloud_cut);
	  out_remove.setNegative(false);
	  out_remove.setMeanK(50);
	  out_remove.setStddevMulThresh(1.0);
	  out_remove.filter(cloud_noise);

	  pcl::toROSMsg (cloud_noise, ocloud);
	  sensor_msgs::convertPointCloud2ToPointCloud(ocloud, out_cloud);
	  out_clusters.push_back(out_cloud);
  }
  sensor_msgs::PointCloud2 big_cluster;
  sensor_msgs::PointCloud bcluster;
  bcluster = out_clusters.at(0);
  sensor_msgs::convertPointCloudToPointCloud2(bcluster, big_cluster);
  //big_cluster=pc2_clusters.at(0);
  big_cluster.header = in_cloud.header;
  first_cluster_pub.publish(big_cluster);

  ROS_INFO("Cluster converted from PointCloud2 array to PointCloud array");
  seg_response.clusters=out_clusters;

//MAKE THE TABLE ////////////////////////////////////////
  // Step 1 : Filter, remove NaNs and downsample
  pcl::PointCloud<Point>::Ptr cloud_ptr (new pcl::PointCloud<Point>);
  pcl::fromROSMsg (in_cloud, *cloud_ptr);
  pcl::PassThrough<Point> pass_;
  pass_.setInputCloud (cloud_ptr);
  pass_.setFilterFieldName ("z");
  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  pcl::PointCloud<Point>::Ptr z_cloud_filtered_ptr (new pcl::PointCloud<Point>);
  pass_.filter (*z_cloud_filtered_ptr);

  pass_.setInputCloud (z_cloud_filtered_ptr);
  pass_.setFilterFieldName ("y");
  pass_.setFilterLimits (y_filter_min_, y_filter_max_);
  pcl::PointCloud<Point>::Ptr y_cloud_filtered_ptr (new pcl::PointCloud<Point>);
  pass_.filter (*y_cloud_filtered_ptr);

  pass_.setInputCloud (y_cloud_filtered_ptr);
  pass_.setFilterFieldName ("x");
  pass_.setFilterLimits (x_filter_min_, x_filter_max_);
  pcl::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl::PointCloud<Point>);
  pass_.filter (*cloud_filtered_ptr);

  pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr (new pcl::PointCloud<Point>);
  pcl::VoxelGrid<Point> grid_;
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_.setFilterFieldName ("z");
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (*cloud_downsampled_ptr);

  // Step 2 : Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<Point>::Ptr normals_tree_;
  normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
  // Normal estimation parameters
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  n3d_.setKSearch (10);
  n3d_.setSearchMethod (normals_tree_);
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (*cloud_normals_ptr);
  ROS_INFO("Normal Estimation done");

  // Step 3 : Perform planar segmentation
  pcl::PointIndices::Ptr table_inliers_ptr (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr table_coefficients_ptr (new pcl::ModelCoefficients);
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  // Table model fitting parameters
  seg_.setDistanceThreshold (0.05);
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);
  seg_.setInputCloud (cloud_downsampled_ptr);
  seg_.setInputNormals (cloud_normals_ptr);
  seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);

  if (table_coefficients_ptr->values.size () <=3)
  {
	ROS_INFO("Failed to detect table in scan");
	seg_response.result = seg_response.NO_TABLE;
	return;
  }

  if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_)
  {
	ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(),
			 inlier_threshold_);
	seg_response.result = seg_response.NO_TABLE;
	return;
  }

  ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].",
			(int)table_inliers_ptr->indices.size (),
			table_coefficients_ptr->values[0], table_coefficients_ptr->values[1],
			table_coefficients_ptr->values[2], table_coefficients_ptr->values[3]);
  ROS_INFO("Planar segmentation done");

  pcl::PointCloud<Point>::Ptr table_projected_ptr (new pcl::PointCloud<Point>);
  pcl::ProjectInliers<Point> proj_;
  proj_.setModelType (pcl::SACMODEL_PLANE);
  proj_.setInputCloud (cloud_downsampled_ptr);
  proj_.setIndices (table_inliers_ptr);
  proj_.setModelCoefficients (table_coefficients_ptr);
  proj_.filter (*table_projected_ptr);
  tf::Transform table_plane_trans;
  table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_, false);

  sensor_msgs::PointCloud table_points;
  if (!getPlanePoints<Point> (*table_projected_ptr, table_plane_trans, table_points))
  {
    seg_response.result = seg_response.OTHER_ERROR;
    return;
  }

  seg_response.table = getTable<sensor_msgs::PointCloud>(in_cloud.header, table_plane_trans, table_points);
  seg_response.result = seg_response.SUCCESS;
}

void MantisSegmentor::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
    {
      visualization_msgs::Marker delete_marker;
      delete_marker.header.stamp = ros::Time::now();
      delete_marker.header.frame_id = frame_id;
      delete_marker.id = id;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      delete_marker.ns = "tabletop_node";
      marker_pub_.publish(delete_marker);
    }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}

tf::Transform MantisSegmentor::getPlaneTransform (pcl::ModelCoefficients coeffs,
		double up_direction, bool flatten_plane)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  tf::Vector3 position(-a*d, -b*d, -c*d);
  tf::Vector3 z(a, b, c);

  //if we are flattening the plane, make z just be (0,0,up_direction)
  if(flatten_plane)
  {
    ROS_INFO("flattening plane");
    z[0] = z[1] = 0;
    z[2] = up_direction;
  }
  else
  {
    //make sure z points "up"
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
    {
      z = -1.0 * z;
      ROS_INFO("flipped z");
    }
  }
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  tf::Vector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
  tf::Vector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  tf::Matrix3x3 rotation;
  rotation[0] = x; 	// x
  rotation[1] = y; 	// y
  rotation[2] = z; 	// z
  rotation = rotation.transpose();
  tf::Quaternion orientation;
  rotation.getRotation(orientation);
  ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
  ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  return tf::Transform(orientation, position);
}
template <typename PointT>
bool MantisSegmentor::getPlanePoints (const pcl::PointCloud<PointT> &table,
		     const tf::Transform& table_plane_trans,
		     sensor_msgs::PointCloud &table_points)
{
  // Prepare the output
  table_points.header = table.header;
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp,
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
	      table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  int current_try=0, max_tries = 3;
  while (1)
  {
    bool transform_success = true;
    try
    {
      listener.transformPointCloud("table_frame", table_points, table_points);
    }
    catch (tf::TransformException ex)
    {
      transform_success = false;
      if ( ++current_try >= max_tries )
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
                  table_points.header.frame_id.c_str(), ex.what());
        return false;
      }
      //sleep a bit to give the listener a chance to get a new transform
      ros::Duration(0.1).sleep();
    }
    if (transform_success) break;
  }
  table_points.header.stamp = table.header.stamp;
  table_points.header.frame_id = "table_frame";
  return true;
}


template <class PointCloudType>
tabletop_object_detector::Table MantisSegmentor::getTable(std_msgs::Header cloud_header,
                                  const tf::Transform &table_plane_trans,
                                  const PointCloudType &table_points)
{
	tabletop_object_detector::Table table;

  //get the extents of the table
  if (!table_points.points.empty())
  {
    table.x_min = table_points.points[0].x;
    table.x_max = table_points.points[0].x;
    table.y_min = table_points.points[0].y;
    table.y_max = table_points.points[0].y;
  }
  for (size_t i=1; i<table_points.points.size(); ++i)
  {
    if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
  }

  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  table.pose.pose = table_pose;
  table.pose.header = cloud_header;


  visualization_msgs::Marker tableMarker = tabletop_object_detector::MarkerGenerator::getTableMarker(table.x_min, table.x_max,
                                                                           table.y_min, table.y_max);
  tableMarker.header = cloud_header;
  tableMarker.pose = table_pose;
  tableMarker.ns = "tabletop_node";
  tableMarker.id = current_marker_id_++;
  marker_pub_.publish(tableMarker);

  return table;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mantis_segmentation");
  ros::NodeHandle nh;

  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("/dominant_plane",1);
  bound_pub = nh.advertise<sensor_msgs::PointCloud2>("/bounded_scene",1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/pre_clustering",1);
  first_cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/biggest_cluster",1);
  //ros::ServiceServer serv = n.advertiseService("/segmentation", segment_cb);




  MantisSegmentor node(nh);

  ros::spin();
  return 0;

}
