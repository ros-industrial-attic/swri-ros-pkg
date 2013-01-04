//test for mantis_object_recognition node
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include <flann/flann.h>

#include <boost/filesystem.hpp>

#include "cph.h"
#include "mantis_perception/mantis_recognition.h"
#include "mantis_perception/mantis_segmentation.h"
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"
#include "tabletop_object_detector/TabletopSegmentation.h"
#include "mantis_data_collection/marker.h"

ros::ServiceClient segmentation_client, marker_client;


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_recognition_test");
  ros::NodeHandle n;

  marker_client = n.serviceClient<mantis_data_collection::marker>("/data_collect_mesh");
  segmentation_client = n.serviceClient<mantis_perception::mantis_segmentation>("/tabletop_segmentation", true);

  //seg_pub = n.advertise<sensor_msgs::PointCloud2>("/segmentation_result",1);

  tf::TransformListener listener;
  geometry_msgs::PointStamped object_point;
  ROS_INFO("Ready");
  while (ros::ok())
  {


    mantis_perception::mantis_segmentation seg_srv;
    if (!segmentation_client.call(seg_srv))
    {
      ROS_ERROR("Call to mantis segmentation service failed");
    }

    sensor_msgs::PointCloud2 segcluster;
    sensor_msgs::PointCloud clustervector;
    clustervector=seg_srv.response.clusters[0];
    sensor_msgs::convertPointCloudToPointCloud2(clustervector, segcluster);
    segcluster.header.frame_id=seg_srv.response.table.pose.header.frame_id;

    mantis_data_collection::marker mark_trans_srv;
    //mark_trans_srv.request.clusters = seg_srv.response.clusters;
    int ang = atoi(argv[1]);
    ROS_INFO_STREAM("char angle input: "<< argv[1]);
    ROS_INFO_STREAM("int angle input: "<< ang);
    mark_trans_srv.request.angle = ang;
    if (!marker_client.call(mark_trans_srv))
    {
      ROS_ERROR("Call to data_collection marker service failed");
    }

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
	  sensor_msgs::PointCloud received_cluster;
	  received_cluster=seg_srv.response.clusters.at(0);
	  sensor_msgs::PointCloud2 cluster_pc2;
	  sensor_msgs::convertPointCloudToPointCloud2(received_cluster, cluster_pc2);
	  pcl::fromROSMsg(cluster_pc2, *cluster);
	  ROS_INFO("Cloud converted");
	  //Demean the cloud.
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid (*cluster, centroid);
	  ROS_INFO("Centroid computed");
	  object_point.header.frame_id = "/base_link";
	  object_point.header.stamp = ros::Time();
	  object_point.point.x = centroid(0);
	  object_point.point.y = centroid(1);
	  object_point.point.z = centroid(2);
	  try{
		geometry_msgs::PointStamped base_point;
		listener.transformPoint("/object_training_frame", object_point, base_point);

		ROS_INFO("base_link: (%.4f, %.4f. %.4f) -----> object_frame: (%.6f, %.6f, %.6f) at time %.2f",
				object_point.point.x, object_point.point.y, object_point.point.z,
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
	  }
	  catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"pvct_training_frame\" to \"base_link\": %s", ex.what());
	  }

    ros::spinOnce();
  }//end ros ok while loop

}




