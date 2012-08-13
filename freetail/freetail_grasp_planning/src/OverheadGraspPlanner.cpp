/*
 * OverheadGraspPlanner.cpp
 *
 *  Created on: Aug 10, 2012
 *      Author: jnicho
 */

#include <planners/OverheadGraspPlanner.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>

const std::string OverheadGraspPlanner::_GraspPlannerName = GRASP_PLANNER_NAME;

typedef pcl::PointCloud<pcl::PointXYZ> PclCloud;
OverheadGraspPlanner::OverheadGraspPlanner() {
	// TODO Auto-generated constructor stub
	fetchParameters(true);
}

OverheadGraspPlanner::~OverheadGraspPlanner() {
	// TODO Auto-generated destructor stub
}

void OverheadGraspPlanner::fetchParameters(bool useNodeNamespace)
{
	std::string paramNamespace = "";
	if(useNodeNamespace)
	{
		paramNamespace = "/" + ros::this_node::getName();
	}
	_ParamVals = ParameterVals();

	ros::param::param(paramNamespace + PARAM_NAME_WORLD_FRAME_ID,_ParamVals.WorldFrameId,
			PARAM_DEFAULT_WORLD_FRAME_ID);
	ros::param::param(paramNamespace + PARAM_NAME_DEFAULT_PREGRASP_DISTANCE,_ParamVals.DefaultPregraspDistance,
			PARAM_DEFAULT_PREGRASP_DISTANCE);
	ros::param::param(paramNamespace + PARAM_NAME_SEARCH_RADIUS_FROM_TOP_POINT,_ParamVals.SearchRadiusFromTopPoint,
			PARAM_DEFAULT_SEARCH_RADIUS_FROM_TOP_POINT);

	// checking if param containing z vector values exists
	XmlRpc::XmlRpcValue list;
	_UsingDefaultApproachVector = true;
	_ParamVals.ApproachVector = PARAM_DEFAULT_APPROACH_VECTOR;
	if(ros::param::has(paramNamespace + PARAM_NAME_APPROACH_VECTOR))
	{
		ros::param::get(paramNamespace + PARAM_NAME_APPROACH_VECTOR,list);

		// additional error checking
		bool valid = true;
		std::string warnExpr;
		valid = list.getType() != XmlRpc::XmlRpcValue::TypeArray;
		ROS_WARN_COND(!valid,"%s",std::string("Invalid data type for '"+ paramNamespace + PARAM_NAME_APPROACH_VECTOR + "', using default").c_str());
		valid = list.size()!= 3;
		ROS_WARN_COND(!valid,"%s",std::string("Incorrect size for '"+ paramNamespace + PARAM_NAME_APPROACH_VECTOR + "', using default").c_str());
		if(valid)
		{
			// converting into vector3 object
			for(int i = 0; i < list.size(); i++)
			{
				if(list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
				{
					ROS_WARN("%s",std::string("Value in '" + paramNamespace + PARAM_NAME_APPROACH_VECTOR + "'is invalid, using default").c_str());
					valid = false;
					_ParamVals.ApproachVector = PARAM_DEFAULT_APPROACH_VECTOR;
					break;
				}
				_ParamVals.ApproachVector.m_floats[i] = static_cast<double>(list[i]);
			}
		}
		_UsingDefaultApproachVector = !valid;
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
	PclCloud cloud = PclCloud(),transformedCloud = PclCloud();
	pcl::fromROSMsg(inCloudMsg,cloud);

	// -------------------------------- resolving cluster reference frame in world coordinates
	tf::TransformListener tfListener;
	tf::StampedTransform clusterTf;
	std::string clusterFrameId = arg.request.target.reference_frame_id;
	try
	{
		tfListener.lookupTransform(_ParamVals.WorldFrameId,clusterFrameId
				,ros::Time(0),clusterTf);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	// -------------------------------- transforming to alternate approach vector
	if(!_UsingDefaultApproachVector)
	{
		// use cros-product and matrix inverse in order to compute transform with modified approach vector (z-direction)
		// use the negative of the modified z-direction vector for all computations.
	}

	// -------------------------------- transforming to correct reference frame --------------------------------------
	Eigen::Affine3d tfEigen;
	tf::TransformTFToEigen(clusterTf,tfEigen);
	pcl::transformPointCloud(cloud,cloud,Eigen::Affine3f(tfEigen));

	// finding bounding box
	pcl::PointXYZ pointMax, pointMin;
	pcl::getMinMax3D(cloud,pointMin,pointMax);

	// extracting highest point from bounding box
	double maxZ = pointMax.z;

	// projecting points onto plane perpendicular to approach vector and placed at highest point
	// check link: http://pointclouds.org/documentation/tutorials/project_inliers.php to see example of projecting onto a plane

	// find all points within search radius
	// check Euclidean Cluster extraction to find points within a certain search radius

	// finding centroid of projected point cloud (should work well for regularly shaped objects)

}


