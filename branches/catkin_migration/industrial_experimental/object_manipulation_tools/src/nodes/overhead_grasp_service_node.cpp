/*
 * overhead_grasp_service_server.cpp
 *
 *  Created on: Oct 17, 2012
 */

#include <object_manipulation_tools/services/GraspPlannerServer.h>
#include <object_manipulation_tools/grasp_planners/OverheadGraspPlanner.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"grasp_planner_server");
	ros::NodeHandle nh;

	boost::shared_ptr<GraspPlannerInterface> planner(new OverheadGraspPlanner());
	GraspPlannerServer graspPlannerServer = GraspPlannerServer(planner);
	graspPlannerServer.init();

	return 0;
}
