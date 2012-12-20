/*
 * BallPickingDemoNode.cpp
 *
 *  Created on: Sep 21, 2012
 */

#include <freetail_object_manipulation/demos/SimpleManipulationDemo.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"ball_picking_demo",ros::init_options::NoSigintHandler);

	SimpleManipulationDemo demo;
	demo.runBallPickingDemo();

	ros::shutdown();

	return 0;
}
