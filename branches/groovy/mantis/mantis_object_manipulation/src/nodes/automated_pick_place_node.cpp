/*
 * automated_pick_place_node.cpp
 *
 *  Created on: Nov 7,
 */

#include <mantis_object_manipulation/arm_navigators/AutomatedPickerRobotNavigator.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"automated_pick_place_node");
	ros::NodeHandle nh;

	ROS_INFO_STREAM(ros::this_node::getName()<<": Creating navigator");
	RobotNavigator::Ptr navigator(new AutomatedPickerRobotNavigator());

	ROS_INFO_STREAM(ros::this_node::getName()<<": Navigator started");
	navigator->run();
	return 0;
}
