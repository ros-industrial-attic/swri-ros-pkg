/*
 * test_sphere_segmentation.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: jnicho
 */

#include <ros/ros.h>
#include <freetail_object_manipulation/segmentation/SphereSegmentation.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>

const std::string TABLETOP_SEGMETATION_SERVICE = "tabletop_segmentation";
const std::string MARKER_TOPIC = "sphere";
const std::string CLUSTER_TOPIC = "camera/depth_registered/points";

void createMarker(const arm_navigation_msgs::CollisionObject &obj,visualization_msgs::Marker &marker)
{
	const arm_navigation_msgs::Shape &shape = obj.shapes[0];

	marker.header.frame_id = obj.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = obj.poses[0];
	marker.scale.x = 2*shape.dimensions[0];// 2 x radius
	marker.scale.y =2*shape.dimensions[0];
	marker.scale.z =2*shape.dimensions[0];
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_sphere_segmentation");
	ros::NodeHandle nh;

	std::string nodeName = ros::this_node::getName();
	SphereSegmentation sphereSeg;

	ros::ServiceClient segmentationClient = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(
			TABLETOP_SEGMETATION_SERVICE,true);

	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(nodeName + "/" + MARKER_TOPIC,1);

	// loop rate
	ros::Rate loopRate(0.5f);

	// tabletop segmentation
	tabletop_object_detector::TabletopSegmentation::Request req;
	tabletop_object_detector::TabletopSegmentation::Response res;

	// sphere segmentation results
	arm_navigation_msgs::CollisionObject obj;

	// sphere marker
	visualization_msgs::Marker sphereMarkerMsg;

	// ros parametr
	bool useTabletopSeg = true;


	while(ros::ok())
	{
		if(useTabletopSeg)
		{
			if(!segmentationClient.call(req,res))
			{
				continue;
			}

			if(res.result != tabletop_object_detector::TabletopSegmentation::Response::SUCCESS)
			{
				continue;
			}

			// segment spheres
			//sensor_msgs::PointCloud &cluster = res.clusters[0];

			ROS_INFO_STREAM(nodeName<<": received "<<res.clusters.size()<<" clusters from tabletop segmentation");
			int counter = 0;
			BOOST_FOREACH(sensor_msgs::PointCloud cloud,res.clusters)
			{
				ROS_INFO_STREAM("cluster "<<counter<<" has "<<cloud.points.size()<<" points");
				counter++;
			}

			sphereSeg.fetchParameters("/" + nodeName);
			//if(sphereSeg.segment(cluster,obj))
			int indexToBestCluster = -1;
			if(sphereSeg.segment(res.clusters,obj,indexToBestCluster))
			{
				arm_navigation_msgs::Shape &shape = obj.shapes[0];
				geometry_msgs::Pose &pose = obj.poses[0];

				ROS_INFO_STREAM("\n"<<nodeName<<": Sphere found in cluster "<<indexToBestCluster<<"\n");
				ROS_INFO_STREAM("\tRadius: "<<shape.dimensions[0]<<"\n");
				ROS_INFO_STREAM("\tx: "<<pose.position.x<<", y: "<<pose.position.y<<", z: "<<pose.position.z<<"\n");

			}
			else
			{
				ROS_ERROR_STREAM(nodeName<<": Sphere segmentation did not succeed");
				continue;
			}
		}
		else
		{
			sensor_msgs::PointCloud2::ConstPtr cluster =
			  ros::topic::waitForMessage<sensor_msgs::PointCloud2>(CLUSTER_TOPIC, nh, ros::Duration(3.0));

			if(cluster == NULL)
			{
			  ROS_ERROR_STREAM(nodeName<<": cloud was not received");
			  continue;
			}
			else
			{
				ROS_INFO_STREAM(nodeName<<": received cloud with "<<cluster->data.size()<< "points");
			}

			sphereSeg.fetchParameters("/" + nodeName);
			if(sphereSeg.segment(*cluster,obj))
			{
				arm_navigation_msgs::Shape &shape = obj.shapes[0];
				geometry_msgs::Pose &pose = obj.poses[0];

				ROS_INFO_STREAM("\n"<<nodeName<<": Sphere found:\n");
				ROS_INFO_STREAM("\tRadius: "<<shape.dimensions[0]<<"\n");
				ROS_INFO_STREAM("\tx: "<<pose.position.x<<", y: "<<pose.position.y<<", z: "<<pose.position.z<<"\n");

			}
			else
			{
				ROS_ERROR_STREAM(nodeName<<": Sphere segmentation did not succeed");
				continue;
			}
		}

		createMarker(obj,sphereMarkerMsg);
		pub.publish(sphereMarkerMsg);

		loopRate.sleep();
		//ros::spin();
	}

	return 0;
}
