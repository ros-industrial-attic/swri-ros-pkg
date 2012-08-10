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

class GraspPlannerServer {
public:
	GraspPlannerServer();
	virtual ~GraspPlannerServer();

	void init();
	void serviceCallback(object_manipulation_msgs::GraspPlanning::Request &request,
			object_manipulation_msgs::GraspPlanning::Response &response);


};

#endif /* GRASPPLANNERSERVER_H_ */
