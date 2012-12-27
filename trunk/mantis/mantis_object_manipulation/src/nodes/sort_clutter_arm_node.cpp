/*
 * sort_clutter_arm_node.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: jnicho
 */

#include <mantis_object_manipulation/arm_navigators/SortClutterArmNavigator.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"sort_clutter_arm_node");
	ros::NodeHandle nh;

	ROS_INFO_STREAM(ros::this_node::getName()<<": Creating navigator");
	RobotNavigator::Ptr navigator(new SortClutterArmNavigator());

	ROS_INFO_STREAM(ros::this_node::getName()<<": Navigator started");
	navigator->run();
	return 0;
}
