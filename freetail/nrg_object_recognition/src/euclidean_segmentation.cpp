#include "ros/ros.h"
#include "nrg_object_recognition/segmentation.h"

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

ros::Publisher plane_pub;
ros::Publisher bound_pub;
ros::Publisher cluster_pub;

bool segment_cb(nrg_object_recognition::segmentation::Request &seg_request,
	      nrg_object_recognition::segmentation::Response &seg_response)
{
  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "segmenting image..." << std::endl;
  pcl::fromROSMsg(seg_request.scene, *cloud);
  
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
 float min_x = seg_request.min_x, max_x = seg_request.max_x;
 float min_y = seg_request.min_y, max_y = seg_request.max_y;
 float min_z = seg_request.min_z, max_z = seg_request.max_z;
 
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
  cloud_filtered_pc2.header.frame_id = "/camera_depth_optical_frame";
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
    plane_pc2.header.frame_id = "/camera_depth_optical_frame";
    plane_pub.publish(plane_pc2);
    
    //std::cout << "Removing tabletop..."  << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
    //std::cout << "done."  << std::endl;
  }
  
  pcl::toROSMsg(*cloud_filtered, cloud_filtered_pc2);
  cloud_filtered_pc2.header.frame_id = "/camera_depth_optical_frame";
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
  
  return (1);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle n;  
  
  plane_pub = n.advertise<sensor_msgs::PointCloud2>("/dominant_plane",1);
  bound_pub = n.advertise<sensor_msgs::PointCloud2>("/bounded_scene",1);
  cluster_pub = n.advertise<sensor_msgs::PointCloud2>("/pre_clustering",1);
  ros::ServiceServer serv = n.advertiseService("/segmentation", segment_cb);
  
  ros::spin();
  
}