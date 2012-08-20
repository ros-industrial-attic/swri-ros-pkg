/*
 * SimulatedControllerNode.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: jnicho
 *
 *  Description:
 *  	This node should allow faking a robot controller that communicates with the "joint_trajectory_action" server.
 *  	It subscribes to the topic "command" of type "trajectory_msgs/JointTrajectory"
 *  	It publishes the topic "state" of type "pr2_controllers_msgs/JointTrajectoryControllerState"
 *  	The command and state topics are published and advertised by the joint_trajectory_action server
 */

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

class SimulatedController
{
	SimulatedController()
	{

	}

	~SimulatedController()
	{

	}
};

int main(int argc, char** argv)
{
	return 0;
}
