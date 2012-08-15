/*
 * GraspPlannerServer.cpp
 *
 *  Created on: Aug 9, 2012
 *      Author: jnicho
 */

#include "servers/GraspPlannerServer.h"

GraspPlannerServer::GraspPlannerServer():
_GraspPlanner(0),
_ServiceName(SERVICE_NAME)
{
	// TODO Auto-generated constructor stub
}

GraspPlannerServer::~GraspPlannerServer() {
	// TODO Auto-generated destructor stub
	finish();
}

void GraspPlannerServer::init()
{
	ros::NodeHandle nh;
	_GraspPlanner = new OverheadGraspPlanner();

	_ServiceServer = nh.advertiseService(_ServiceName,&GraspPlannerServer::serviceCallback,this);
	ROS_INFO("%s",std::string(_GraspPlanner->getPlannerName() + " advertising service: " + _ServiceName).c_str());
	ros::spin();
}

void GraspPlannerServer::finish()
{
	if(_GraspPlanner)
	{
		delete _GraspPlanner;
		_GraspPlanner = NULL;
	}
}

bool GraspPlannerServer::serviceCallback(object_manipulation_msgs::GraspPlanning::Request &req,
		object_manipulation_msgs::GraspPlanning::Response &res)
{
	if(_GraspPlanner)
	{
		return _GraspPlanner->planGrasp(req,res);
	}
	else
	{
		return false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"grasp_planner_server");
	ros::NodeHandle nh;

	GraspPlannerServer graspPlannerServer = GraspPlannerServer();
	graspPlannerServer.init();

	return 0;
}
