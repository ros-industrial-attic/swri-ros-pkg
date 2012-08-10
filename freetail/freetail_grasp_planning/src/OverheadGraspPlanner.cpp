/*
 * OverheadGraspPlanner.cpp
 *
 *  Created on: Aug 10, 2012
 *      Author: jnicho
 */

#include "OverheadGraspPlanner.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
typedef pcl::PointCloud<pcl::PointXYZ> PclCloud;
OverheadGraspPlanner::OverheadGraspPlanner() {
	// TODO Auto-generated constructor stub
	fetchParameters(true);
}

OverheadGraspPlanner::~OverheadGraspPlanner() {
	// TODO Auto-generated destructor stub
}

OverheadGraspPlanner::fetchParameters(bool useNodeNamespace)
{
	std::string paramNamespace = "";
	if(useNodeNamespace)
	{
		paramNamespace = "/" + ros::this_node::getName();
	}

}

std::string OverheadGraspPlanner::getPlannerName()
{
	return _GraspPlannerName;
}

bool OverheadGraspPlanner::planGrasp(object_manipulation_msgs::GraspPlanning &arg)
{
	// converting message to pcl type
	sensor_msgs::PointCloud2 inCloudMsg;
	sensor_msgs::convertPointCloudToPointCloud2(arg.request.target.cluster,inCloudMsg);
	PclCloud cloud = PclCloud();;
	pcl::fromROSMsg(inCloudMsg,cloud);

	// transforming to correct reference frame

	// finding bounding box

	// extracting highest point from bounding box

	// projecting points onto plane perpendicular to z and placed at highest point

	// finding centroid of projected point cloud (should work well for regularly shaped objects)

}


