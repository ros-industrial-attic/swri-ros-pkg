/*
 * test_recognition_node.cpp
 *
 *  Created on: Aug 7, 2012
 *      Author: jnicho
 */
#include <freetail_object_manipulation/SimpleManipulationDemo.h>

// ros parameters
const std::string PARAM_NAME_LOOP_RATE = "loop_rate";
const std::string PARAM_NAME_PAUSE_LOOP = "pause_node";

void testVfHSegmentation(ros::NodeHandle &nodeHandle)
{
	SimpleManipulationDemo demo;
	std::string nodeName = ros::this_node::getName() + "/";

	// fetching parameters
	double loopRateVal = 4.0f;
	bool pauseLoop = false;
	ros::param::param(nodeName + PARAM_NAME_LOOP_RATE,loopRateVal,loopRateVal);
	ros::param::param(nodeName + PARAM_NAME_LOOP_RATE,pauseLoop,pauseLoop);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	srand(time(NULL));

	ros::Rate loopRate(loopRateVal);
	while(ros::ok())
	{
		if(!pauseLoop)
		{
			demo.startCycleTimer();
			if(!demo.segmentAndRecognize())
			{
				ROS_ERROR("Recognition failed");
			}

			loopRate.sleep();
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

