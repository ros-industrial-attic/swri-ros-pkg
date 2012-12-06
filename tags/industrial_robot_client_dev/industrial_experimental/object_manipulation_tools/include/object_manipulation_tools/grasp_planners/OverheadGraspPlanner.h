/*
 * OverheadGraspPlanner.h
 *
 *  Created on: Aug 10, 2012
 *      Author: jnicho
 */

#ifndef OVERHEADGRASPPLANNER_H_
#define OVERHEADGRASPPLANNER_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <object_manipulation_tools/grasp_planners/GraspPlannerInterface.h>
#include <XmlRpc.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// ros parameters bare names
const std::string PARAM_NAME_DEFAULT_PREGRASP_DISTANCE = "/default_pregrasp_distance";
const std::string PARAM_NAME_PLANE_PROXIMITY_THRESHOLD= "/plane_proximity_threshold";
const std::string PARAM_NAME_NUM_CANDIDATE_GRASPS = "/num_returned_candidate_grasps";
const std::string PARAM_NAME_GRASP_IN_WORLD_COORDINATES = "grasp_pose_in_world_coordinates";
const std::string PARAM_NAME_SEARCH_RADIUS = "/search_radius";
const std::string PARAM_NAME_APPROACH_VECTOR = "/approach_vector";


// default ros parameter values
const double PARAM_DEFAULT_PREGRASP_DISTANCE = 0.1f;// meters
const double PARAM_DEFAULT_PLANE_PROXIMITY_THRESHOLD= 0.005; // meters
const double PARAM_DEFAULT_SEARCH_RADIUS = 0.005; //0.5 cm
const int PARAM_DEFAULT_NUM_CANDIDATE_GRASPS = 8;
const bool PARAM_DEFAULT_GRASP_IN_WORLD_COORDINATES = true; //
const tf::Vector3 PARAM_DEFAULT_APPROACH_VECTOR = tf::Vector3(0.0f,0.0f,-1.0f);  // in world coordinates

// other defaults
const std::string GRASP_PLANNER_NAME = "OverheadGraspPlanner";
const int SEARCH_MIN_CLUSTER_SIZE = 50.0f;
const int SEARCH_MAX_CLUSTER_SIZE = 5000;

class OverheadGraspPlanner:public GraspPlannerInterface
{
public:

	struct ParameterVals
	{
		double DefaultPregraspDistance;
		double PlaneProximityThreshold;
		double SearchRadius;
		int NumCandidateGrasps;
		bool GraspInWorldCoordinates;
		tf::Vector3 ApproachVector; // in world coordinates
	};

	OverheadGraspPlanner();
	virtual ~OverheadGraspPlanner();

	bool planGrasp(object_manipulation_msgs::GraspPlanning::Request &req,
			object_manipulation_msgs::GraspPlanning::Response &res);
	std::string getPlannerName();
	void fetchParameters(bool useNodeNamespace);

	// rotates about the approach vector to generate candidate grasp poses
	void generateGraspPoses(const geometry_msgs::Pose &pose,int numCandidates,
			std::vector<geometry_msgs::Pose> &poses);

protected:
	static const std::string _GraspPlannerName;
	std::string _WorldFrameId;
	ParameterVals _ParamVals;
	tf::TransformListener _tfListener;

	// computational members
	bool _UsingDefaultApproachVector;
};

#endif /* OVERHEADGRASPPLANNER_H_ */
