/*
 * OverheadGraspPlanner.cpp
 *
 *  Created on: Aug 10, 2012
 *      Author: jnicho
 */

#include <object_manipulation_tools/grasp_planners/OverheadGraspPlanner.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <tf/LinearMath/Scalar.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>

const std::string OverheadGraspPlanner::_GraspPlannerName = GRASP_PLANNER_NAME;
typedef pcl::PointCloud<pcl::PointXYZ> PclCloud;
OverheadGraspPlanner::OverheadGraspPlanner():
_ParamVals(ParameterVals()),
_WorldFrameId("/NO_PARENT"),
_tfListener()
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
	ros::param::param(paramNamespace + PARAM_NAME_PLANE_PROXIMITY_THRESHOLD,_ParamVals.PlaneProximityThreshold,
			PARAM_DEFAULT_PLANE_PROXIMITY_THRESHOLD);
	ros::param::param(paramNamespace + PARAM_NAME_SEARCH_RADIUS,_ParamVals.SearchRadius,
			PARAM_DEFAULT_SEARCH_RADIUS);
	ros::param::param(paramNamespace + PARAM_NAME_NUM_CANDIDATE_GRASPS,_ParamVals.NumCandidateGrasps,
				PARAM_DEFAULT_NUM_CANDIDATE_GRASPS);
	ros::param::param(paramNamespace + PARAM_NAME_GRASP_IN_WORLD_COORDINATES,_ParamVals.GraspInWorldCoordinates,
				PARAM_DEFAULT_GRASP_IN_WORLD_COORDINATES);

	// checking if param containing z vector values exists
	XmlRpc::XmlRpcValue list;
	_UsingDefaultApproachVector = true;
	_ParamVals.ApproachVector = PARAM_DEFAULT_APPROACH_VECTOR;

	if(ros::param::get(paramNamespace + PARAM_NAME_APPROACH_VECTOR,list) && (list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
			(list.size() > 2) && (list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble))
	{
		double val;
		val = static_cast<double>(list[0]);_ParamVals.ApproachVector.setX(val);
		val = static_cast<double>(list[1]);_ParamVals.ApproachVector.setY(val);
		val = static_cast<double>(list[2]);_ParamVals.ApproachVector.setZ(val);

		_UsingDefaultApproachVector = false;

		tf::Vector3 &v = _ParamVals.ApproachVector;
		ROS_INFO_STREAM("Overhead Grasp Planer found approach vector: ["<<v.x()<<", "<<v.y()<<", "<<v.z()<<"]");
	}
	else
	{
		_UsingDefaultApproachVector = true;

		tf::Vector3 &v = _ParamVals.ApproachVector;
		ROS_ERROR_STREAM("Overhead Grasp Planner could not read/find approach vector under parameter "<< PARAM_NAME_APPROACH_VECTOR<<
				", will use default vector: ["<<v.x()<<", "<<v.y()<<", "<<v.z()<<"] instead");
	}

}

std::string OverheadGraspPlanner::getPlannerName()
{
	return _GraspPlannerName;
}

bool OverheadGraspPlanner::planGrasp(object_manipulation_msgs::GraspPlanning::Request &req,
		object_manipulation_msgs::GraspPlanning::Response &res)
{
	// updating parameters
	fetchParameters(true);

	// checking if grasp to evaluate were passed
	if(!req.grasps_to_evaluate.empty())
	{
		object_manipulation_msgs::Grasp candidateGrasp;
		std::vector<geometry_msgs::Pose> candidatePoses;
		sensor_msgs::JointState jointState;
		jointState.name = std::vector<std::string>();
		jointState.position = std::vector<double>();
		jointState.velocity = std::vector<double>();
 		candidateGrasp.grasp_pose = geometry_msgs::Pose();

 		// generating candidate poses
		candidateGrasp.grasp_posture = jointState;
		candidateGrasp.pre_grasp_posture = jointState;
		candidateGrasp.desired_approach_distance = _ParamVals.DefaultPregraspDistance;
		candidateGrasp.min_approach_distance = _ParamVals.DefaultPregraspDistance;
 		for(std::size_t i = 0;i < req.grasps_to_evaluate.size(); i++)
 		{
 			object_manipulation_msgs::Grasp &graspEval = req.grasps_to_evaluate[i];
 			geometry_msgs::Pose &poseToEval = graspEval.grasp_pose;
 			generateGraspPoses(poseToEval,_ParamVals.NumCandidateGrasps,candidatePoses);
 		}

 		// storing all candidate poses
 		for(std::size_t i = 0; i < candidatePoses.size(); i++)
 		{
 			candidateGrasp.grasp_pose = candidatePoses[i];
 			res.grasps.push_back(candidateGrasp);
 		}

 		res.error_code.value = object_manipulation_msgs::GraspPlanningErrorCode::SUCCESS;
 		return true;
	}

	// local variables
	ros::NodeHandle nh;
	std::stringstream stdOut;

	// converting message to pcl type
	sensor_msgs::PointCloud2 inCloudMsg;
	sensor_msgs::convertPointCloudToPointCloud2(req.target.cluster,inCloudMsg);
	PclCloud cloud = PclCloud(),transformedCloud = PclCloud();
	pcl::fromROSMsg(inCloudMsg,cloud);

	// ---------------------------------------------------------------------------------------------------------------
	// -------------------------------- resolving cluster reference frame in world coordinates
	tf::StampedTransform clusterTf; clusterTf.setIdentity();// will be modified if alternate approach direction is used
	tf::StampedTransform clusterTfInWorld; // will remain fixed
	std::string clusterFrameId = req.target.cluster.header.frame_id;
	_WorldFrameId = req.target.reference_frame_id;
	try
	{
		_tfListener.lookupTransform(_WorldFrameId ,clusterFrameId,
				ros::Time(0),clusterTf);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",std::string(getPlannerName() + " , failed to resolve transform from " +
				_WorldFrameId + " to " + clusterFrameId + " \n\t\t" + " tf error msg: " +  ex.what()).c_str());
		ROS_WARN("%s",std::string(getPlannerName() + ": Will use Identity as cluster transform").c_str());
		clusterTf.setData(tf::Transform::getIdentity());
	}
	clusterTfInWorld.setData(clusterTf);

	// ---------------------------------------------------------------------------------------------------------------
	// -------------------------------- transforming to alternate approach vector
	// This transformation will allow finding the highest point relative to the opposite direction of the approach vector
	tf::Transform approachTf;approachTf.setIdentity();
	if(!_UsingDefaultApproachVector)
	{
		// use cros-product and matrix inverse in order to compute transform with modified z-direction (approach vector)
		// use the negative of the modified z-direction vector for all computations.
		tf::Matrix3x3 rotMat;rotMat.setIdentity();
		tf::Vector3 zVec = -_ParamVals.ApproachVector.normalize();

		// checking that both z vectors have different directions
		tfScalar tolerance = 0.01f;
		tfScalar angle = zVec.angle(tf::Vector3(0.0f,0.0f,1.0f));
		if(std::abs(angle) > tolerance)
		{
			tf::Vector3 xVec = tf::Vector3((zVec.cross(tf::Vector3(0.0f,0.0f,1.0f))).normalize()); //z_approach x z_world (0,0,1)
			tf::Vector3 yVec = tf::Vector3((zVec.cross(xVec)).normalize());
			rotMat.setValue(xVec.x(),yVec.x(),zVec.x(),
					xVec.y(),yVec.y(),zVec.y(),
					xVec.z(),yVec.z(),zVec.z());
			approachTf = tf::Transform(rotMat,tf::Vector3(0.0f,0.0f,0.0f));

			stdOut<< getPlannerName() << ": Approach transform found is:";
			for(int i = 0; i < 3; i++)
			{
				tf::Vector3 row = approachTf.getBasis().getRow(i);
				stdOut<< "\n\t" << "|\t" << row.x() << ",\t" << row.y() << ",\t" << row.z()<<",\t" << approachTf.getOrigin()[i]<<"\t|";
			}
			ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

			// transforming cluster transform to new coordinates
			clusterTf.setData((approachTf.inverse())*(tf::Transform)clusterTf);
			stdOut << getPlannerName() << ": Aligned the input cluster to the transform corresponding to the modified approach vector";
			ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
		}
		else
		{
			stdOut << getPlannerName() << ": Requested approach vector and world z-vector are too close, using default world frame";
			ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
		}
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
	stdOut << getPlannerName() << ": Found Min Point at x: "<<pointMin.x<<", y: "<<pointMin.y<<", z: " << pointMin.z;
	ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
	stdOut << getPlannerName() << ": Found Max Point at x: "<<pointMax.x<<", y: "<<pointMax.y<<", z: " << maxZ;
	ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

	// ---------------------------------------------------------------------------------------------------------------
	// finding points near or on top plane
	PclCloud::Ptr planeCloud  = boost::make_shared<PclCloud>();
	pcl::PassThrough<pcl::PointXYZ> passFilter;
	passFilter.setInputCloud(boost::make_shared<PclCloud>(cloud));
	passFilter.setFilterFieldName("z");
	passFilter.setFilterLimits(maxZ - std::abs(_ParamVals.PlaneProximityThreshold),
			maxZ + std::abs(_ParamVals.PlaneProximityThreshold));
	passFilter.filter(*planeCloud);
	if(planeCloud->size() > 0)
	{
		stdOut << getPlannerName() << ": Found " << planeCloud->size()<<" points at a proximity of "
				<<_ParamVals.PlaneProximityThreshold << " to top plane";
		// printing found points
		BOOST_FOREACH(pcl::PointXYZ point,*planeCloud)
		{
			stdOut<<"\n\t"<<"x: "<<point.x<<", y: "<<point.y<<", z: "<<point.z;
		}
		ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
	}
	else
	{
		stdOut<<getPlannerName()<< ": Found not points on top plane, canceling request";
		ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
		return false;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// projecting filtered points onto top plane
	// defining plane coefficients
	tf::Vector3 normal = tf::Vector3(0.0f,0.0f,1.0f);
	double offset = -normal.dot(tf::Vector3(0.0f,0.0f,maxZ));
	pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = normal.getX();
	coefficients->values[1] = normal.getY();
	coefficients->values[2] = normal.getZ();
	coefficients->values[3] = offset;
	stdOut << getPlannerName() << ": Created Contact Plane with coefficients: " << coefficients->values[0]
		   << ", "	<< coefficients->values[1] << ", " << coefficients->values[2] << ", " << coefficients->values[3];
	ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

	// projecting points
	PclCloud::Ptr projectedCloud = boost::make_shared<PclCloud>();
	pcl::ProjectInliers<pcl::PointXYZ> projectObj;
	projectObj.setModelType(pcl::SACMODEL_PLANE);
	projectObj.setInputCloud(planeCloud);
	projectObj.setModelCoefficients(coefficients);
	projectObj.filter(*projectedCloud);


	// finding all points within a search radius
	/*	This search attempts to find points that belong to the same object while removing those that are not part
	 * of the object of interest but were found to be close enough to the plane.
	*/
	if(_ParamVals.SearchRadius > 0) // skip if search radius <= 0 and use all points instead
	{
		bool continueSearch = true;
		unsigned int index = 0;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
		std::vector<int> indices;
		std::vector<float> sqDistances;
		searchTree->setInputCloud(projectedCloud);

		while(continueSearch)
		{
			int found = searchTree->radiusSearch(projectedCloud->points[index],
					_ParamVals.SearchRadius,indices,sqDistances);

			if(found > 0)
			{
				continueSearch = false;
			}
			else
			{
				index++;
				if(index == projectedCloud->points.size())
				{
					ROS_ERROR_STREAM(getPlannerName() << ": Did not find points within search radius: " <<
							_ParamVals.SearchRadius <<",  exiting");

					res.grasps = std::vector<object_manipulation_msgs::Grasp>();
					res.error_code.value = object_manipulation_msgs::GraspPlanningErrorCode::OTHER_ERROR;

					return false;
				}
			}
		}

		// extracting points
		pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
		inliers->indices = indices;
		pcl::ExtractIndices<pcl::PointXYZ> extractObj;
		extractObj.setInputCloud(projectedCloud);
		extractObj.setIndices(inliers);
		extractObj.setNegative(false);
		extractObj.filter(*projectedCloud);

		stdOut << getPlannerName() << ": Found " << projectedCloud->size() << " points within search radius: " <<
				_ParamVals.SearchRadius;
		ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
	}
	else
	{
		ROS_INFO_STREAM(getPlannerName()<<": search radius <= 0, skipping search and using all points found instead");
	}

	// finding centroid of projected point cloud (should work well for objects with relative degree of symmetry)
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*projectedCloud,centroid);
	stdOut << getPlannerName() << ": Found centroid at: x = " << centroid[0] << ", y = " << centroid[1] << ", z = " << centroid[2];
	ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

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

//	object_manipulation_msgs::GraspPlanningErrorCode errorCode;
//	errorCode.value = errorCode.SUCCESS;

	res.grasps = grasps;
	res.error_code.value = object_manipulation_msgs::GraspPlanningErrorCode::SUCCESS;
	return true;
}

void OverheadGraspPlanner::generateGraspPoses(const geometry_msgs::Pose &pose,int numCandidates,
		std::vector<geometry_msgs::Pose> &poses)
{
	tf::Transform graspTf = tf::Transform::getIdentity();
	tf::Transform candidateTf;
	tfScalar angle = tfScalar(2*M_PI/(double(numCandidates)));

	// converting initial pose to tf
	tf::poseMsgToTF(pose,graspTf);

	for(int i = 0; i < numCandidates; i++)
	{
		candidateTf = graspTf*tf::Transform(tf::Quaternion(_ParamVals.ApproachVector,i*angle),
				tf::Vector3(0.0f,0.0f,0.0f));
//		candidateTf = graspTf;
//		candidateTf.setRotation(tf::Quaternion(_ParamVals.ApproachVector,i*angle));
		geometry_msgs::Pose candidatePose = geometry_msgs::Pose();
		tf::poseTFToMsg(candidateTf,candidatePose);
		poses.push_back(candidatePose);
	}
}


