/*
 * GraspPlannerInterface.h
 *
 *  Created on: Aug 10, 2012
 *      Author: jnicho
 */

#ifndef GRASPPLANNERINTERFACE_H_
#define GRASPPLANNERINTERFACE_H_

#include <object_manipulation_msgs/GraspPlanning.h>
#include <string.h>

class GraspPlannerInterface
{
public:
	virtual bool planGrasp(object_manipulation_msgs::GraspPlanning::Request &req,
			object_manipulation_msgs::GraspPlanning::Response &res) = 0;
	virtual std::string getPlannerName() = 0;
	virtual void fetchParameters(bool useNodeNamespace) = 0;

protected:
};

#endif /* GRASPPLANNERINTERFACE_H_ */
