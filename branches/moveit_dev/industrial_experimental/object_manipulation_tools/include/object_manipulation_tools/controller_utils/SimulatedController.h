/*
 * SimulatedController.h
 *
 *  Created on: Oct 10, 2012
 *      Author: coky
 */

/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SIMULATEDCONTROLLER_H_
#define SIMULATEDCONTROLLER_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <boost/foreach.hpp>

const std::string CONTROLLER_STATE_TOPIC_NAME = "state";
const std::string CONTROLLER_FEEDBACK_TOPIC_NAME = "feedback_states";
const std::string JOINT_TRAJECTORY_TOPIC_NAME = "command";
const std::string JOINT_STATE_TOPIC_NAME = "joint_states";
const std::string JOINT_NAMES_PARAM_NAME = "/joint_trajectory_action/joints";

class SimulatedController
{
public:
	SimulatedController();

	~SimulatedController();

	void init();

	void broadcastState(const ros::TimerEvent &evnt);

	void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

protected:

	void updateLastStateFeedbackMessages(const sensor_msgs::JointState &st);

	// ros comm
	ros::Subscriber _JointTrajSubscriber;
	ros::Publisher _ControllerStatePublisher;
	ros::Publisher _JointStatePublisher;
	ros::Publisher _ControllerFeedbackPublisher;
	ros::Timer _StatusUpdateTimer;

	// topic names
	std::string _CntrlStatePubTopic;
	std::string _ControlFeedbackPubTopic;
	std::string _JointStatePubTopic;
	std::string _JointTrajSubsTopic;

	// ros parameters
	std::vector<std::string> _JointNames;

	// last states
	sensor_msgs::JointState _LastJointState;
	control_msgs::FollowJointTrajectoryFeedback _LastControllerTrajState;
	pr2_controllers_msgs::JointTrajectoryControllerState _LastControllerJointState;

	// others
	bool _ProcessingRequest;
};

#endif /* SIMULATEDCONTROLLER_H_ */
