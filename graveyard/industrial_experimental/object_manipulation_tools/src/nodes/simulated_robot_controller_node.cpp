/*
 * SimulatedControllerNode.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: jnicho
 *
 *  Description:
 *  	This node should allows simulating a robot controller that communicates with the "joint_trajectory_action" server.
 *  	It subscribes to the topic "command" of type "trajectory_msgs/JointTrajectory"
 *  	It publishes the topic "state" of type "pr2_controllers_msgs/JointTrajectoryControllerState"
 *  	The command and state topics are published and advertised by the joint_trajectory_action server
 */

#include <ros/ros.h>
#include <object_manipulation_tools/controller_utils/SimulatedController.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"simulated_robot_controller_node");
	ros::NodeHandle nh;
	SimulatedController simController = SimulatedController();
	simController.init();
	return 0;
}
