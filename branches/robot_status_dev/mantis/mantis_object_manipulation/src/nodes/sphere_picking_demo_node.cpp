/*
 * sphere_picking_demo_node.cpp
 *
 *  Created on: Oct 19, 2012
 */
#include <mantis_object_manipulation/arm_navigators/SpherePickingRobotNavigator.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pick_place_demo_node");
	ros::NodeHandle nh;

	ROS_INFO_STREAM(ros::this_node::getName()<<": Creating navigator");
	RobotNavigator::Ptr navigator(new SpherePickingRobotNavigator());

	ROS_INFO_STREAM(ros::this_node::getName()<<": Navigator started");
	navigator->run();
	return 0;
}
