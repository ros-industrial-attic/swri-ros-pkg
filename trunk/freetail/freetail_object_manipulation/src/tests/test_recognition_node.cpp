/*
 * test_recognition_node.cpp
 *
 *  Created on: Aug 7, 2012
 *      Author: jnicho
 */
#include <freetail_object_manipulation/demos/SimpleManipulationDemo.h>

// ros parameters
const std::string PARAM_NAME_LOOP_RATE = "loop_rate";
const std::string PARAM_NAME_PAUSE_LOOP = "pause_node";

void testVfHSegmentation(ros::NodeHandle &nodeHandle)
{
	ROS_INFO("Instantiating Manipulation Demo obj");
	SimpleManipulationDemo demo;

	ROS_INFO("Setting up object recognition");
	demo.setupRecognitionOnly();
	std::string nodeName = ros::this_node::getName() + "/";

	// fetching parameters
	double loopRateVal = 4.0f;
	bool pauseLoop = false;
	ros::param::param(nodeName + PARAM_NAME_LOOP_RATE,loopRateVal,loopRateVal);
	ros::param::param(nodeName + PARAM_NAME_LOOP_RATE,pauseLoop,pauseLoop);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	srand(time(NULL));

	while(ros::ok())
	{
		if(!pauseLoop)
		{
			demo.startCycleTimer();
			if(!demo.recognize())
			{
				ROS_ERROR("Recognition failed");
			}

			ros::Rate(loopRateVal).sleep();
		}
	}

	ros::waitForShutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"test_segmentation_node");
	ros::NodeHandle nh;
	testVfHSegmentation(nh);
	return 0;
}

