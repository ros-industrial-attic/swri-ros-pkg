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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>

#include "/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopSegmentation.h"
#include </opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopObjectRecognition.h>
/**
 * This node will subscribe to point cloud data from the Kinect and perform segment objects down to ping pong balls
 */

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pubt;
ros::Publisher pubs;
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
  pub2.publish (clusters);

//definitely not the table...
/*  sensor_msgs::PointCloud table;
  sensor_msgs::PointCloud2 tablecluster;
  int end = segmentation_srv.response.clusters.size()-1;
  table=segmentation_srv.response.clusters[end];
  sensor_msgs::convertPointCloudToPointCloud2(table, tablecluster);
  tablecluster.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  tablecluster.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  pubt.publish(tablecluster);
*/
  //TAKE CLUSTER OF BIN/BALL AND REMOVE RED (BIN)
  pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb;// (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(clusters,PCxyzrgb);

  pcl::PointCloud<pcl::PointXYZI> PCxyzi;
  pcl::PointCloudXYZRGBtoXYZI(PCxyzrgb, PCxyzi);
  sensor_msgs::PointCloud2 clusterint;

  pcl::toROSMsg(PCxyzi, clusterint);
  clusterint.header.frame_id="/base_link";
  clusterint.header.stamp=ros::Time::now();
   //publish clusters for rviz
   pub3.publish (clusterint);

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
*/	  //convert RGB to HSV
	  float r, g, b;
	  r=r1/255.0; g=g1/255.0; b=b1/255.0;
	  float M, m, C;
	  M=max(max(r,g),b);
	  m=min(min(r,g),b);
	  C=M-m;
/*
	  ROS_INFO("Current chroma value: %4.2f", (float)C);
	  ROS_INFO("Current r value: %4.2f", (float)r);
	  ROS_INFO("Current g value: %4.2f", (float)g);
	  ROS_INFO("Current b value: %4.2f", (float)b);
	  //ROS_INFO("Current max value: %4.2f", (float)M);
*/
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

	  if (V==0.0)
	  {
		  S=0.0;
	  }
	  else
	  {
		  S=C/V;
	  }
	  float I=(r+g+b)/3.0;
	  float S3=1-(m/I);
	  double alpha = 0.5*(2*r-g-b);
	  double beta = (sqrt(3)/2.0)*(g-b);
	  double H2 = atan2(beta, alpha);
/*
	  ROS_INFO("Current H value: %4.2f", (float)H);
	  ROS_INFO("Current V value: %4.2f", (float)V);
	  ROS_INFO("Current S value: %4.2f", (float)S);

	  double alpha = 0.5*(2*r-g-b);
	  double beta = (sqrt(3)/2)*(g-b);
	  double H2 = atan2(beta, alpha);
	  double C2 = sqrt((alpha*alpha)+(beta*beta));
	  float S2, V2;
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
*/
	  //if H~0 and S~1 and v~1, then the points should be red
	  //if (H<=90 &&  V>=0.8)//S>=0.6 &&
	  //if (H2<=30 && S2>=0.6 && V>=0.2)
	  //if (r>0.5 && g<0.15 && b<0.15)
	  //if (r1>=100 && g1<=50 && b1<=50)
	  //if (CHr>=0.5 && CHr <= 1.0 && CHg<=0.15)
	  if (H2<=1 && H2>=-1)//&& S3>0.95)//  && H2>=-0.15)// && S3<0.1)H2<=0.01
	  {
		  ivec.push_back(count);

		  /*ROS_INFO("Current H2 value: %4.2f", (float)H2);
		  ROS_INFO("Current I value: %4.2f", (float)I);
		  ROS_INFO("Current S3 value: %4.2f", (float)S3);*/
	  }

  }
  for (int i=0; i < ivec.size(); i++)
    {
	  PCxyzrgb.erase(PCxyzrgb.begin()+ivec[i]);
    }

  //ROS_INFO("random point in vector at position 40 %d", (double)alpha);
  ROS_INFO("Cluster of balls - width %d", (int)PCxyzrgb.width);

  sensor_msgs::PointCloud2 ballcluster;
  pcl::toROSMsg(PCxyzrgb, ballcluster);

  //publish clusters for rviz
  pub.publish (ballcluster);


  //FIT SPHERES
  pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(ballcluster,*PCxyz);
  //pcl::PointCloud<pcl::PointXYZ> PCxyz;
  //pcl::PointCloudXYZRGBtoXYZ(PCxyzrgb, PCxyz);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ> final;
  std::vector<int> inliers;
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
      model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (PCxyz));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
  ransac.setDistanceThreshold (.005);
  ransac.computeModel();
  ransac.getInliers(inliers);

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*PCxyz, inliers, *final);


  sensor_msgs::PointCloud2 spheres;
  pcl::toROSMsg(*final,spheres);
  //publish clusters for rviz
  pubs.publish (spheres);


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
  pub = n.advertise<sensor_msgs::PointCloud2>("ball_cluster", 100);
  pub2 = n.advertise<sensor_msgs::PointCloud2>("bin_cluster", 100);
  pub3 = n.advertise<sensor_msgs::PointCloud2>("cluster_intensity", 100);
  pubt = n.advertise<sensor_msgs::PointCloud2>("table", 100);
  pubs = n.advertise<sensor_msgs::PointCloud2>("spheres", 100);

  ros::spin();

  return EXIT_SUCCESS;
}
