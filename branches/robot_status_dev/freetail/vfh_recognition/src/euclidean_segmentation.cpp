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
#include <vfh_recognition/SupportClasses.h>


int
SegmentCloud(sensor_msgs::PointCloud2 rawCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloudSegments, pcl::PointCloud<pcl::PointXYZ>::Ptr table, RosParametersList &params)
{
  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(rawCloud, *cloud);
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_0 (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  std::cout << "leaf sizes from param server: " << params.Vals.SegmentationLeafSizeX << std::endl;
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
//   vg.setLeafSize (params.Vals.SegmentationLeafSizeX,
// 		  params.Vals.SegmentationLeafSizeY,
// 		  params.Vals.SegmentationLeafSizeZ);
  vg.filter (*cloud_filtered_0);
  
  //std::cout << "Points remaining after voxel grid filter: " << cloud_filtered_0->points.size() << std::endl;
  
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
  seg.setDistanceThreshold (0.02);
//   seg.setMaxIterations (params.Vals.SegmentationMaxIterations);
//   seg.setDistanceThreshold (params.Vals.SegmentationDistanceThreshold);
  
  //Spatial filter.
  cloud_filtered->resize(0);
  //Parameters
 float min_x = -.7, max_x = .7;
 float min_y = -.1, max_y = 10;
 float min_z = 0, max_z = 1.2;
  
//   float min_x = params.Vals.SegmentationSpatialFilterMinX;
//   float max_x = params.Vals.SegmentationSpatialFilterMaxX;
//   float min_y = params.Vals.SegmentationSpatialFilterMinY;
//   float max_y = params.Vals.SegmentationSpatialFilterMaxY;
//   float min_z = params.Vals.SegmentationSpatialFilterMinZ;
//   float max_z = params.Vals.SegmentationSpatialFilterMaxZ;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator position=cloud_filtered_0->begin(); position!=cloud_filtered_0->end(); position++){
    if(position->x > min_x && position->x < max_x && position->y > min_y && position->y < max_y && position->z > min_z && position->z < max_z)
      cloud_filtered->push_back(*position);
  }
  int nr_points = (int) cloud_filtered->points.size ();
  //std::cout << "Points after spatial filter: " << nr_points << std::endl;
  
  float acctPercentage = params.Vals.SegmentationClusterConfigAcctPercnt;
  while (cloud_filtered->points.size () > 0.3 * nr_points)
//   while (cloud_filtered->points.size () > acctPercentage * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);    
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
  }
  //*table = *cloud_plane;
  std::cout << "Points after spatial filter: " << cloud_filtered->points.size() << std::endl;
  if(cloud_filtered->points.size() <= 100){
   ROS_INFO("Insufficient points remaining after filtering. Segmentation failed.");
   return -1;
  }
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
//   ec.setClusterTolerance (params.Vals.SegmentationClusterConfigSpatialTolerance);
//   ec.setMinClusterSize (params.Vals.SegmentationClusterConfigMinSize);
//   ec.setMaxClusterSize (params.Vals.SegmentationClusterConfigMaxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::cout << "Clusters found: " << cluster_indices.size() << std::endl;
  
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloudSegments.push_back(cloud_cluster);
    j++;
  }
  return (1);
}

int
SegmentCloud(sensor_msgs::PointCloud2 rawCloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloudSegments)
{
	RosParametersList params = RosParametersList();
	params.loadParams(true);
	pcl::PointCloud<pcl::PointXYZ>::Ptr table;
	return SegmentCloud(rawCloud,cloudSegments,table, params);
}

