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
#include <tf/LinearMath/Scalar.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>

const std::string OverheadGraspPlanner::_GraspPlannerName = GRASP_PLANNER_NAME;
typedef pcl::PointCloud<pcl::PointXYZ> PclCloud;
OverheadGraspPlanner::OverheadGraspPlanner():
_ParamVals(ParameterVals()),
_WorldFrameId("/NO_PARENT")
{
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

	ros::param::param(paramNamespace + PARAM_NAME_DEFAULT_PREGRASP_DISTANCE,_ParamVals.DefaultPregraspDistance,
			PARAM_DEFAULT_PREGRASP_DISTANCE);
	ros::param::param(paramNamespace + PARAM_NAME_SEARCH_RADIUS_FROM_TOP_POINT,_ParamVals.SearchRadiusFromTopPoint,
			PARAM_DEFAULT_SEARCH_RADIUS_FROM_TOP_POINT);
	ros::param::param(paramNamespace + PARAM_NAME_NUM_CANDIDATE_GRASPS,_ParamVals.NumCandidateGrasps,
				PARAM_DEFAULT_NUM_CANDIDATE_GRASPS);
	ros::param::param(paramNamespace + PARAM_NAME_GRASP_IN_WORLD_COORDINATES,_ParamVals.GraspInWorldCoordinates,
				PARAM_DEFAULT_GRASP_IN_WORLD_COORDINATES);

	// checking if param containing z vector values exists
	XmlRpc::XmlRpcValue list;
	_UsingDefaultApproachVector = true;
	_ParamVals.ApproachVector = PARAM_DEFAULT_APPROACH_VECTOR;

	// additional error checking
	bool valid = false;
	_UsingDefaultApproachVector = !valid;
	_ParamVals.ApproachVector = _ParamVals.ApproachVector.normalize();
	if(ros::param::has(paramNamespace + PARAM_NAME_APPROACH_VECTOR))
	{
		ros::param::get(paramNamespace + PARAM_NAME_APPROACH_VECTOR,list);

		std::string warnExpr;
		valid = list.getType() == XmlRpc::XmlRpcValue::TypeArray;
		if(!valid)
		{
			ROS_WARN("%s",std::string("Invalid data type for '" + paramNamespace + PARAM_NAME_APPROACH_VECTOR + "', using default").c_str());
			return;
		}

		valid = list.size()== 3;
		if(!valid)
		{
			ROS_WARN("%s",std::string("Incorrect size for '"+ paramNamespace + PARAM_NAME_APPROACH_VECTOR + "', using default").c_str());
			return;
		}

		_UsingDefaultApproachVector = !valid;
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
					_UsingDefaultApproachVector = !valid;
					return;
				}
				_ParamVals.ApproachVector.m_floats[i] = static_cast<double>(list[i]);
			}
		}
	}
}

std::string OverheadGraspPlanner::getPlannerName()
{
	return _GraspPlannerName;
}

bool OverheadGraspPlanner::planGrasp(object_manipulation_msgs::GraspPlanning::Request &req,
		object_manipulation_msgs::GraspPlanning::Response &res)
{
	// converting message to pcl type
	sensor_msgs::PointCloud2 inCloudMsg;
	sensor_msgs::convertPointCloudToPointCloud2(req.target.cluster,inCloudMsg);
	PclCloud cloud = PclCloud(),transformedCloud = PclCloud();
	pcl::fromROSMsg(inCloudMsg,cloud);

	// ---------------------------------------------------------------------------------------------------------------
	// -------------------------------- resolving cluster reference frame in world coordinates
	tf::TransformListener tfListener;
	tf::StampedTransform clusterTf;// will be modified if alternate approach direction is used
	tf::StampedTransform clusterTfInWorld; // will remain fixed
	std::string clusterFrameId = req.target.cluster.header.frame_id;
	_WorldFrameId = req.target.reference_frame_id;
	try
	{
		tfListener.lookupTransform(_WorldFrameId ,clusterFrameId
				,ros::Time(0),clusterTf);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",std::string(getPlannerName() + " , failed to resolve transform from" +
				_WorldFrameId + " to " + clusterFrameId + "/n/t/t" + " tf error msg: " +  ex.what()).c_str());
		ROS_ERROR("%s",std::string(getPlannerName() + ": Will use Identity as cluster transform").c_str());
		clusterTf.setData(tf::Transform::getIdentity());
	}
	clusterTfInWorld.setData(clusterTf);

	// ---------------------------------------------------------------------------------------------------------------
	// -------------------------------- transforming to alternate approach vector
	// This transformation will allow finding the highest point relative to the opposite direction of the approach vector
	tf::Transform approachTf = tf::Transform::getIdentity();
	if(!_UsingDefaultApproachVector)
	{
		// use cros-product and matrix inverse in order to compute transform with modified z-direction (approach vector)
		// use the negative of the modified z-direction vector for all computations.
		tf::Matrix3x3 rotMat;
		tf::Vector3 zVec = -_ParamVals.ApproachVector;
		tf::Vector3 xVec = zVec.cross(tf::Vector3(0.0f,0.0f,1.0f)); //z_approach x z_world (0,0,1)
		tf::Vector3 yVec = zVec.cross(xVec);
		rotMat.setValue(xVec.x(),yVec.x(),zVec.x(),
				xVec.y(),yVec.y(),zVec.y(),
				xVec.z(),yVec.z(),zVec.z());
		approachTf = tf::Transform(rotMat,tf::Vector3(0.0f,0.0f,0.0f));

		// transforming cluster transform to new coordinates
		clusterTf.setData((approachTf.inverse())*(tf::Transform)clusterTf);
	}

	// ---------------------------------------------------------------------------------------------------------------
	// -------------------------------- transforming to correct reference frame --------------------------------------
	Eigen::Affine3d tfEigen;
	tf::TransformTFToEigen(clusterTf,tfEigen);
	pcl::transformPointCloud(cloud,cloud,Eigen::Affine3f(tfEigen));

	// finding bounding box
	pcl::PointXYZ pointMax, pointMin;
	pcl::getMinMax3D(cloud,pointMin,pointMax);

	// extracting highest point from bounding box
	double maxZ = pointMax.z;

	// ---------------------------------------------------------------------------------------------------------------
	// projecting points onto plane perpendicular to approach vector and placed at highest point

	// defining plane coefficients
	tf::Vector3 normal = tf::Vector3(0.0f,0.0f,1.0f);
	double offset = -normal.dot(tf::Vector3(0.0f,0.0f,maxZ));
	pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = normal.getX();
	coefficients->values[1] = normal.getY();
	coefficients->values[2] = normal.getZ();
	coefficients->values[3] = offset;

	// projecting points
	PclCloud::Ptr projectedCloud = boost::make_shared<PclCloud>();
	pcl::ProjectInliers<pcl::PointXYZ> projectObj;
	projectObj.setModelType(pcl::SACMODEL_PLANE);
	projectObj.setInputCloud(boost::make_shared<PclCloud>(cloud));
	projectObj.setModelCoefficients(coefficients);
	projectObj.filter(*projectedCloud);

	// finding all points within a search radius
	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	std::vector<int> indices;
	std::vector<float> sqDistances;
	searchTree->setInputCloud(projectedCloud);
	searchTree->radiusSearch(pcl::PointXYZ(0.0f,0.0f,maxZ),_ParamVals.SearchRadiusFromTopPoint,indices,sqDistances);

	// extracting points
	pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
	inliers->indices = indices;
	pcl::ExtractIndices<pcl::PointXYZ> extractObj;
	extractObj.setInputCloud(projectedCloud);
	extractObj.setIndices(inliers);
	extractObj.setNegative(false);
	extractObj.filter(*projectedCloud);

	// finding centroid of projected point cloud (should work well for objects with relative degree of symmetry)
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*projectedCloud,centroid);

	// ---------------------------------------------------------------------------------------------------------------
	// -------------------------------- generating candidate grasps  --------------------------------
	geometry_msgs::Pose  graspPose;
	sensor_msgs::JointState jointsGrasp;
	sensor_msgs::JointState jointsPreGrasp;
	jointsPreGrasp.name = jointsGrasp.name = std::vector<std::string>();
	jointsPreGrasp.position = jointsGrasp.position = std::vector<double>();

	// creating first grasp pose
	tf::Transform graspTf;
	tf::Quaternion rot = tf::Quaternion::getIdentity();
	rot.setRotation(tf::Vector3(1.0f,0.0f,0.0f),M_PI); // rotates 180 about x in order to invert the direction of the approach vector (z-vector)
	graspTf.setOrigin(tf::Vector3(centroid[0],centroid[1],centroid[2]));
	graspTf.setRotation(rot);

	if(_UsingDefaultApproachVector)
	{
		graspTf = approachTf*graspTf; // transforming to world coordinates
	}

	if(!_ParamVals.GraspInWorldCoordinates) // transform to object coordinates
	{
		graspTf = clusterTfInWorld.inverse()*graspTf;
	}

	// creating grasp array
	std::vector<object_manipulation_msgs::Grasp> grasps;

	// computing additional candidate transforms
	if(_ParamVals.NumCandidateGrasps > 1)
	{
		int numGrasps = _ParamVals.NumCandidateGrasps;
		tfScalar angle = tfScalar(2*M_PI/(double(numGrasps)));
		tf::Transform rotatedGraspTf = graspTf;
		for(int i = 1; i < numGrasps - 1;i++)
		{
			// rotating by (angle x i) about z axis of initial grasp pose
			rotatedGraspTf.setRotation(rotatedGraspTf.getRotation()*tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),angle*i));

			// filling candidate grasp message data
			object_manipulation_msgs::Grasp candidateGrasp;
			candidateGrasp.grasp_pose = geometry_msgs::Pose();
			tf::quaternionTFToMsg(graspTf.getRotation(),candidateGrasp.grasp_pose.orientation);
			candidateGrasp.grasp_pose.position.x = rotatedGraspTf.getOrigin().getX();
			candidateGrasp.grasp_pose.position.y = rotatedGraspTf.getOrigin().getY();
			candidateGrasp.grasp_pose.position.z = rotatedGraspTf.getOrigin().getZ();
			candidateGrasp.grasp_posture = jointsGrasp;
			candidateGrasp.pre_grasp_posture = jointsPreGrasp;
			candidateGrasp.desired_approach_distance = _ParamVals.DefaultPregraspDistance;
			candidateGrasp.min_approach_distance = _ParamVals.DefaultPregraspDistance;

			// adding grasp
			grasps.push_back(candidateGrasp);
		}
	}
	else
	{
		object_manipulation_msgs::Grasp candidateGrasp;
		candidateGrasp.grasp_pose = geometry_msgs::Pose();
		tf::quaternionTFToMsg(graspTf.getRotation(),candidateGrasp.grasp_pose.orientation);
		candidateGrasp.grasp_pose.position.x = graspTf.getOrigin().getX();
		candidateGrasp.grasp_pose.position.y = graspTf.getOrigin().getY();
		candidateGrasp.grasp_pose.position.z = graspTf.getOrigin().getZ();
		candidateGrasp.grasp_posture = jointsGrasp;
		candidateGrasp.pre_grasp_posture = jointsPreGrasp;
		candidateGrasp.desired_approach_distance = _ParamVals.DefaultPregraspDistance;
		candidateGrasp.min_approach_distance = _ParamVals.DefaultPregraspDistance;

		// adding grasp
		grasps.push_back(candidateGrasp);
	}

	object_manipulation_msgs::GraspPlanningErrorCode errorCode;
	errorCode.value = errorCode.SUCCESS;

	res.grasps = grasps;
	res.error_code = errorCode;
	return true;
}


