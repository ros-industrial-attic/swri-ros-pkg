/*
 * GraspPlannerServer.h
 *
 *  Created on: Aug 9, 2012
 *      Author: jnicho
 */

#ifndef GRASPPLANNERSERVER_H_
#define GRASPPLANNERSERVER_H_

#include <ros/ros.h>
#include <object_manipulator/object_manipulator.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <ros/package.h>
#include <planners/OverheadGraspPlanner.h>

const std::string SERVICE_NAME = "plan_point_cluster_grasp";

class GraspPlannerServer {
public:
	GraspPlannerServer();
	virtual ~GraspPlannerServer();
	void init();
	void finish();
	bool serviceCallback(object_manipulation_msgs::GraspPlanning::Request &request,
			object_manipulation_msgs::GraspPlanning::Response &response);

protected:

	GraspPlannerInterface *_GraspPlanner;
	ros::ServiceServer _ServiceServer;
	std::string _ServiceName;
};

#endif /* GRASPPLANNERSERVER_H_ */
