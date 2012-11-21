#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include "mantis_data_collection/marker.h"
#include "mantis_perception/mantis_segmentation.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher vis_pub;
ros::ServiceClient seg_client;

bool mesh_cb(mantis_data_collection::marker::Request &req, mantis_data_collection::marker::Response &res)
{
/*
	static tf::TransformBroadcaster broadcaster;

    broadcaster.sendTransform(
	      tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.545, -0.18, 0.043)),
	        ros::Time::now(),"/base_link", "/object_training_frame"));
*/
  float theta = req.angle*3.14159/180;//radians
  ROS_INFO("Angle input successfully");
	visualization_msgs::Marker mesh_marker;
	mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	//mesh_marker.type = visualization_msgs::Marker::SPHERE;
	mesh_marker.action = visualization_msgs::Marker::ADD;
	mesh_marker.lifetime = ros::Duration();
	mesh_marker.header.frame_id = "base_link";
	//note: data collection to this point has been done in a way such that a response rotation of 225
	//corresponds to a table frame rotation of 0.
	//good z value for pvc_t=0.0317044
	//good z value for pvc_elbow z: 0.0662595-0.025 (should be a tiny bit higher)
	mesh_marker.header.stamp= ros::Time();
	mesh_marker.scale.x = 1;
	mesh_marker.scale.y = 1;
	mesh_marker.scale.z = 1;
	mesh_marker.pose.position.x = 0.545;
	mesh_marker.pose.position.y = -0.18;
	mesh_marker.pose.position.z = 0.049;
	mesh_marker.pose.orientation.x = 0.0;
	mesh_marker.pose.orientation.y = 0.0;
	//need to convert from angle representation to quaternion representation
	mesh_marker.pose.orientation.z = sin(theta/2);
	mesh_marker.pose.orientation.w = cos(theta/2);
	mesh_marker.color.a = 1.0;
	mesh_marker.color.r = 0.0;
	mesh_marker.color.g = 1.0;
	mesh_marker.color.b = 0.0;
	mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/elec_enclosure.STL";
	vis_pub.publish( mesh_marker );

	ROS_INFO("Finished publishing mesh");
/*
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
	  sensor_msgs::PointCloud received_cluster;
	  received_cluster=req.clusters.at(0);
	  sensor_msgs::PointCloud2 cluster_pc2;
	  sensor_msgs::convertPointCloudToPointCloud2(received_cluster, cluster_pc2);
	  pcl::fromROSMsg(cluster_pc2, *cluster);
	ROS_INFO("Cloud converted");
	  //Demean the cloud.
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid (*cluster, centroid);
	  ROS_INFO("Centroid computed");

  geometry_msgs::PointStamped base_point;
  base_point.header.frame_id = "/base_link";
  base_point.header.stamp = ros::Time();
  base_point.point.x = centroid(0);
  base_point.point.y = centroid(1);
  base_point.point.z = centroid(2);
  tf::TransformListener listener;
  geometry_msgs::PointStamped object_point;;

  ROS_INFO_STREAM("base_link: "<<base_point.point.x<< "\n" <<base_point.point.y<<"\n" << base_point.point.z);


	  try{

		listener.waitForTransform("/object_training_frame", "/base_link",
		                              ros::Time::now(), ros::Duration(3.0));

		listener.transformPoint("/object_training_frame", base_point, object_point);

		ROS_INFO("object_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
				object_point.point.x, object_point.point.y, object_point.point.z,
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
	  }
	  catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"object_training_frame\": %s", ex.what());
	  }
	res.points.at(0) = object_point.point.x;
	res.points.at(1) = object_point.point.y;
	res.points.at(2) = object_point.point.z;*/

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "marker_transform_node");
	ros::NodeHandle n;
	ROS_INFO("Ready!");

	ros::ServiceServer marker_serv;
	marker_serv = n.advertiseService("/data_collect_mesh", mesh_cb);
	//seg_client =n.serviceClient<mantis_perception::mantis_segmentation>("/mantis_segmentation");
	vis_pub = n.advertise<visualization_msgs::Marker>( "matching_mesh_marker", 10 );

	ros::spin();

	return 0;
}


