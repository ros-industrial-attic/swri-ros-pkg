#include "ros/ros.h"
#include <algorithm>
using namespace std;

#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <boost/foreach.hpp>
#include "ros/console.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include "/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopSegmentation.h"
#include "/home/cgomez/ros/fuerte/swri-ros-pkg/freetail/freetail_ball_bin_segmentation/srv_gen/cpp/include/freetail_ball_bin_segmentation/BallBinSegmentation.h"

/**
 * This node will subscribe to point cloud data from the Kinect and perform segment objects down to ping pong balls
 */

/*ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pubt;
ros::Publisher pubs;
ros::ServiceClient rec_srv;*/
ros::ServiceClient seg_srv;
ros::ServiceClient ball_srv;

//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void pointcloudcallback(const sensor_msgs::PointCloud2ConstPtr& pcmsg)
{
	freetail_ball_bin_segmentation::BallBinSegmentation ballbin_srv;
	      if (!ball_srv.call(ballbin_srv))
	         {
	            ROS_ERROR("Call to segmentation service failed");
	            //return false;
	          }
	      if (ballbin_srv.response.result != ballbin_srv.response.SUCCESS)
	          {
	            ROS_ERROR("Segmentation service returned error %d", ballbin_srv.response.result);
	            //return false;
	          }

	      //addDetectedTableToPlanningSceneDiff(segmentation_srv.response.table);
	  ROS_INFO("Ball/bin Segmentation service succeeded. Detected ball clusters");
/*
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
  sensor_msgs::PointCloud2 bincluster;
  sensor_msgs::PointCloud clustervector;
  clustervector=segmentation_srv.response.clusters[0];
  sensor_msgs::convertPointCloudToPointCloud2(clustervector, bincluster);
  bincluster.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  bincluster.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  //pub2.publish (bincluster);

  //TAKE CLUSTER OF BIN/BALL AND REMOVE RED (BIN)
  pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb;// (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(bincluster,PCxyzrgb);

  ROS_INFO("Cluster of bin with balls - %d points", (int)PCxyzrgb.width);
  //ROS_INFO("Cluster of bin with balls - height %d", (int)PCxyzrgb.height);
  //width - the total number of points in the cloud (equal with POINTS see below) for unorganized datasets
  int pcsize=PCxyzrgb.width;
  std::vector<int> ivec;
  for (int count=0; count < pcsize; count++)
  {
	  float r1, g1, b1;
	  r1=PCxyzrgb.at(count).r;
	  g1=PCxyzrgb.at(count).g;
	  b1=PCxyzrgb.at(count).b;
/*

	  r1=255; g1=0; b1=0;
	  //convert RGB to HSV
	  float r, g, b;
	  r=r1/255.0; g=g1/255.0; b=b1/255.0;
	  float M, m, C;
	  M=max(max(r,g),b);
	  m=min(min(r,g),b);
	  C=M-m;
*//*
	  ROS_INFO("Current chroma value: %4.2f", (float)C);
	  ROS_INFO("Current r value: %4.2f", (float)r);
	  ROS_INFO("Current g value: %4.2f", (float)g);
	  ROS_INFO("Current b value: %4.2f", (float)b);
	  //ROS_INFO("Current max value: %4.2f", (float)M);

	  float H1, H, V, S;
	  if (C == 0.0)
		{
		  H=0;
		}
		else if (M==r)
		{
		  H=60*((g-b)/C);
		}
		else if (M==g)
		{
		  H=120+60*((b-r)/C);
		}
		else if (M==b)
		{
		  H=240+60*((r-g)/C);
		}
		else
			H=0;

	  if (H<0)
	  {
		  H+=360;
	  }
	  V=M;
      if (C==0.0)
	  {
		  S=0.0;
	  }
	  else
	  {
		  S=C/V;
	  }
*/
/*	  float I=(r+g+b)/3.0;
	  float S3=1-(m/I);

	  double alpha = 0.5*(2*r-g-b);
	  double beta = (sqrt(3)/2.0)*(g-b);
	  double H2 = atan2(beta, alpha);
	  double C2 = sqrt((alpha*alpha)+(beta*beta));


	  ROS_INFO("Current H value: %4.2f", (float)H);
	  ROS_INFO("Current V value: %4.2f", (float)V);
	  ROS_INFO("Current S value: %4.2f", (float)S);
*/
/*	  float S2, V2;
	  V2=M;
	  if (C2==0.0)
	  	  {
	  		  S2=0.0;
	  	  }
	  	  else
	  	  {
	  		  S2=C2/V2;
	  	  }

	  float CHr=r1/(r1+g1+b1);
	  float CHg=g1/(r1+g1+b1);

	  //IF H~0 AND S~1 and V~1, then the points should be red
	  //if (V>=0.4)H<=359)// &&  S>=0.05)// && V>=0.05
	  //if (H2<=30 && S2>=0.6 && V>=0.2)
	  //if (g1<=240)//r1>=100)// && g1<=50 && b1<=50)
	  //if (CHg<=0.4)// && CHg<=0.5)//&& CHg<=0.15
	  if (H2<=1 && H2>=-1 && S2>0.3)//)  && H2>=-0.15)// && S3<0.1)H2<=0.01
	  {
		  ivec.push_back(count);

	  }

  }
  for (int i=0; i < ivec.size(); i++)
    {
	  PCxyzrgb.erase(PCxyzrgb.begin()+ivec[i]);
    }

  ROS_INFO("Cluster of balls - %d points", (int)PCxyzrgb.width);

  sensor_msgs::PointCloud2 ballcluster;
  pcl::toROSMsg(PCxyzrgb, ballcluster);
  ballcluster.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  ballcluster.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  //publish clusters for rviz
  //pub.publish (ballcluster);
*/

/*
  //LESS ELEGANT BUT EFFECTIVE BOUNDARY EROSION (?-Coordinate system)

  float x_max=0.0, y_max=0.0, x_min=1.0, y_min=1.0, z_max=0.0, z_min=1.0;

  pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb2;
  PCxyzrgb2=PCxyzrgb;
  int pcsize2=PCxyzrgb2.width;

  for (int count2=0; count2 < pcsize2; count2++)
    {
  	  float x_coord, y_coord, z_coord;
  	  x_coord=PCxyzrgb2.at(count2).x;
  	  y_coord=PCxyzrgb2.at(count2).y;
  	  z_coord=PCxyzrgb2.at(count2).z;

  	  if (x_coord > x_max)
  	  {
  		  x_max=x_coord;
  	  }
  	  if (y_coord > y_max)
  	  {
  		  y_max=y_coord;
  	  }
  	  if (x_coord < x_min)
  	  {
  		  x_min=x_coord;
  	  }
  	  if (y_coord < y_min)
  	  {
  		  y_min=y_coord;
  	  }
  	  if (z_coord > z_max)
  	  {
  		  z_max=z_coord;
  	  }
  	  if (z_coord < z_min)
  	  {
  		  z_min=z_coord;
  	  }
    }

  ROS_INFO("x min %f", (float)x_min);
  ROS_INFO("x max %f", (float)x_max);
  ROS_INFO("y min %f", (float)y_min);
  ROS_INFO("y max %f", (float)y_max);
  ROS_INFO("z min %f", (float)z_min);
  ROS_INFO("z max %f", (float)z_max);
  std::vector<int> ivec2;
  for (int count3=0; count3 < pcsize2; count3++)
    {
      float x_coord, y_coord, z_coord;
  	  x_coord=PCxyzrgb2.at(count3).x;
  	  y_coord=PCxyzrgb2.at(count3).y;
  	  z_coord=PCxyzrgb2.at(count3).z;
  	  if (z_coord<z_min+0.08)//&&x_coord>x_max-0.2 && y_coord>y_max-0.15x_coord<x_min+0.15 && y_coord<y_min+0.15
  	  {
  		  ivec2.push_back(count3);
  	  }
    }

    for (int j=0; j < ivec2.size(); j++)
      {
  	  PCxyzrgb2.erase(PCxyzrgb2.begin()+ivec2[j]);
      }

    ROS_INFO("Cluster of balls only - %d points", (int)PCxyzrgb2.width);

    sensor_msgs::PointCloud2 balls;
    pcl::toROSMsg(PCxyzrgb2, balls);
    balls.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
    balls.header.stamp=segmentation_srv.response.table.pose.header.stamp;
    //publish clusters for rviz
    pubs.publish (balls);
*/


/*
//BOUNDARY ESTIMATION OF BALL CLOUD
  //pcl::PointCloud<pcl::PointXYZ> PCxyz;
  pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(ballcluster,*PCxyz);
  vector<int> indices;
  boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

  // Estimate normals first
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normest;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

  // set parameters
  normest.setInputCloud (PCxyz);
  //boost::shared_ptr<vector<int> > indicesptr (new vector<int> (indices));
  normest.setIndices (indicesptr);
  normest.setSearchMethod (tree);
  normest.setRadiusSearch (0.03);
  normest.setKSearch (static_cast<int> (indices.size ()));
  // estimate
  normest.compute (*normals);

  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> b;
  b.setInputNormals (normals);

  // Object
  pcl::PointCloud<pcl::Boundary>::Ptr bps (new pcl::PointCloud<pcl::Boundary> ());

  // set parameters
  b.setInputCloud (PCxyz);
  b.setIndices (indicesptr);
  b.setSearchMethod (tree);
  b.setRadiusSearch(.5);
  b.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  b.compute (*bps);

//  sensor_msgs::PointCloud2 edge;
//  sensor_msgs::convertPointCloudToPointCloud2(*bps, edge);
//  edge.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
//  edge.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  pubs.publish (*bps);


//COLOR BASED REGION GROWING SEGMENTATION
  //USES PCL 1.7 WHICH I THOUGHT WAS IN perception_pcl_fuerte_unstable BUT DOES NOT APPEAR TO BE
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (PCxyzrgb);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;

  reg.setInputCloud (PCxyzrgb);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (60);

  std::vector <pcl::PointIndices> colorcluster;
  reg.extract (colorcluster);

  sensor_msgs::PointCloud2 colorclusters;
  sensor_msgs::convertPointCloudToPointCloud2(colorcluster, colorclusters);
  colorclusters.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  colorclusters.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  pubs.publish (colorclusters);



//TRY TO FIT PLANE TO BIN SIDE - NOT SUCCESSFUL
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(ballcluster,*PCxyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.001);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //int nr_points = (int) PCxyz->points.size ();
  //while (PCxyz->points.size () > 0.3 * nr_points)
  //  {
      // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (PCxyz);
  seg.segment (*plane_inliers, *coefficients);
  if (plane_inliers->indices.size () == 0)
      {
        ROS_INFO("Could not estimate a planar model for the given dataset.");
        //break;
      }
      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " "
                                            << coefficients->values[3] << std::endl;


      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (PCxyz);
      extract.setIndices (plane_inliers);
      extract.setNegative (false);

      // Write the planar inliers to disk
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      //cloud_filtered = cloud_f;
  //  }
  sensor_msgs::PointCloud2 ballclusters;
  pcl::toROSMsg (*cloud_f, ballclusters);

  ballclusters.header.frame_id="/base_link";
  ballclusters.header.stamp=ros::Time::now();

  pub3.publish (ballclusters);

//FIT SPHERES -> CURRENTLY SEEMS TO FIT ONE SPHERE TO DATA i.e. no good

  //pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::fromROSMsg(ballcluster,*PCxyz);
  //pcl::PointCloud<pcl::PointXYZ> PCxyz;
  //pcl::PointCloudXYZRGBtoXYZ(PCxyzrgb, PCxyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> inliers;
  std::vector<int> model;
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
      model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (PCxyz));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
  ransac.setDistanceThreshold (.000005);
  ransac.computeModel();
  ransac.getInliers(inliers);
  ROS_INFO("Found %d inliers", (int)inliers.size());
  ransac.getModel(model);
  ROS_INFO("Found %d model(s)", (int)model.size());
  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*PCxyz, inliers, *final);

  sensor_msgs::PointCloud2 spheres;
  pcl::toROSMsg(*final,spheres);
  pubs.publish (spheres);


//INSTEAD OF FIT SPHERES, EXTRACT CLUSTER OF SINGLE BALL -> NOT SUCCESSFUL
  pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(ballcluster,*PCxyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (PCxyz);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 1cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (3000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (PCxyz);
  ec.extract (cluster_indices);

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

      for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
           //push_back: add a point to the end of the existing vector
           cloud_cluster->points.push_back(PCxyz->points[*pit]);
           }
      //Merge current clusters to whole point cloud
      //*clustered_cloud += *cloud_cluster;
  }
  sensor_msgs::PointCloud2 ballclusters;
  pcl::toROSMsg (*cloud_cluster, ballclusters);

  ballclusters.header.frame_id="/base_link";
  ballclusters.header.stamp=ros::Time::now();

  pub3.publish (ballclusters);
*/

  /* SEGMENTATION USING ONLY PCL ALGORITHMS -> not as awesome as the tabletop_segmentation node/service

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
  //seg_srv = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);

  ball_srv = n.serviceClient<freetail_ball_bin_segmentation::BallBinSegmentation>("/ballbin_segmentation", true);

/*  freetail_ball_bin_segmentation::BallBinSegmentation ballbin_srv;
      if (!ball_srv.call(ballbin_srv))
         {
            ROS_ERROR("Call to segmentation service failed");
            //return false;
          }
      if (ballbin_srv.response.result != ballbin_srv.response.SUCCESS)
          {
            ROS_ERROR("Segmentation service returned error %d", ballbin_srv.response.result);
            //return false;
          }

      //addDetectedTableToPlanningSceneDiff(segmentation_srv.response.table);
  ROS_INFO("Ball/bin Segmentation service succeeded. Detected ball clusters");


  //tabletop_object_detector::TabletopSegmentation segmentation;
  //segment(segmentation);


  //publish topic for rviz visualization

  pub2 = n.advertise<sensor_msgs::PointCloud2>("bin_cluster", 100);
  pub3 = n.advertise<sensor_msgs::PointCloud2>("cluster_intensity", 100);
  pubt = n.advertise<sensor_msgs::PointCloud2>("table", 100);
  pubs = n.advertise<sensor_msgs::PointCloud2>("spheres", 100);
pub = n.advertise<sensor_msgs::PointCloud2>("ball_cluster", 100);
*/  ros::spin();

  return EXIT_SUCCESS;
}
