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
class OverheadGraspPlanner:public GraspPlannerInterface
{
public:
	OverheadGraspPlanner();
	virtual ~OverheadGraspPlanner();

	bool planGrasp(object_manipulation_msgs::GraspPlanning &arg);
	std::string getPlannerName();
	void fetchParameters(bool useNodeNamespace);

protected:
	const std::string _GraspPlannerName = "OverheadGraspPlanner";
};

#endif /* OVERHEADGRASPPLANNER_H_ */
