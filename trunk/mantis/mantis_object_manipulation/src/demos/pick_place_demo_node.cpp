/*
 * arm_pick_place_demo.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: coky
 */
#include <arm_navigators/RobotPickPlaceNavigator.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pick_place_demo_node");
	ros::NodeHandle nh;
	RobotPickPlaceNavigator navigator(RobotPickPlaceNavigator::SETUP_SPHERE_PICK_PLACE);
	navigator.run();
	return 0;
}
