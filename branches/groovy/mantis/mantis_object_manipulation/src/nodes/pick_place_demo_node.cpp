/*
 * arm_pick_place_demo.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: jnicho
 */
#include <mantis_object_manipulation/arm_navigators/RobotPickPlaceNavigator.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pick_place_demo_node");
	ros::NodeHandle nh;

	ROS_INFO_STREAM(ros::this_node::getName()<<": Creating navigator");
	RobotPickPlaceNavigator navigator(RobotPickPlaceNavigator::SETUP_SPHERE_PICK_PLACE);

	ROS_INFO_STREAM(ros::this_node::getName()<<": Navigator started");
	navigator.run();
	return 0;
}
