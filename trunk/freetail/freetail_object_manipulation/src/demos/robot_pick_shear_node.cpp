/*
 * robot_pick_shear_node.cpp
 *
 *  Created on: Nov 2, 2012
 */

#include <ros/ros.h>
#include <freetail_object_manipulation/arm_navigators/RobotGripperNavigator.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"robot_pick_shear_node");

	ROS_INFO_STREAM(ros::this_node::getName()<<": Creating navigator");
	boost::shared_ptr<RobotNavigator> navigator(new RobotGripperNavigator());
	ROS_INFO_STREAM(ros::this_node::getName()<<": Starting navigator");
	navigator->run();
}

