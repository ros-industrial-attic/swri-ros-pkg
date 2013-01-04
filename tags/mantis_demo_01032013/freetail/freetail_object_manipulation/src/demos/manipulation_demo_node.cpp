/*
 * manipulation_demo_node.cpp
 *
 *  Created on: Aug 7, 2012
 *      Author: jnicho
 */
#include <freetail_object_manipulation/demos/SimpleManipulationDemo.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"manipulation_demo",ros::init_options::NoSigintHandler);

	SimpleManipulationDemo demo;
	demo.runSimpleManipulationDemo();

	ros::shutdown();

	return 0;
}
