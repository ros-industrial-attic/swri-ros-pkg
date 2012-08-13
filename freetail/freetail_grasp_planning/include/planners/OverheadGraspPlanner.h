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
#include <planners/GraspPlannerInterface.h>
#include <tf/transform_datatypes.h>
#include <XmlRpc.h>

// ros parameters bare names
const std::string PARAM_NAME_WORLD_FRAME_ID = "/world_frame_id";
const std::string PARAM_NAME_DEFAULT_PREGRASP_DISTANCE = "/default_pregrasp_distance";
const std::string PARAM_NAME_SEARCH_RADIUS_FROM_TOP_POINT = "/search_radius_from_top_point";
const std::string PARAM_NAME_APPROACH_VECTOR = "APPROACH_VECTOR";

// default ros parameter values
const std::string PARAM_DEFAULT_WORLD_FRAME_ID = "/NO_PARENT";
const double PARAM_DEFAULT_PREGRASP_DISTANCE = 0.1f;// meters
const double PARAM_DEFAULT_SEARCH_RADIUS_FROM_TOP_POINT = 0.01; // meters
const tf::Vector3 PARAM_DEFAULT_APPROACH_VECTOR = tf::Vector3(0.0f,0.0f,-1.0f);  // in world coordinates

// other defaults
const std::string GRASP_PLANNER_NAME = "OverheadGraspPlanner";

class OverheadGraspPlanner:public GraspPlannerInterface
{
public:

	struct ParameterVals
	{
		std::string WorldFrameId;
		double DefaultPregraspDistance;
		double SearchRadiusFromTopPoint;
		tf::Vector3 ApproachVector; // in world coordinates
	};

	OverheadGraspPlanner();
	virtual ~OverheadGraspPlanner();

	bool planGrasp(object_manipulation_msgs::GraspPlanning &arg);
	std::string getPlannerName();
	void fetchParameters(bool useNodeNamespace);

protected:
	static const std::string _GraspPlannerName;
	ParameterVals _ParamVals;

	// computational members
	bool _UsingDefaultApproachVector;
};

#endif /* OVERHEADGRASPPLANNER_H_ */
