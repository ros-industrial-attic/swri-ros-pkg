/*
* SimulatedController.cpp
*
*  Created on: Oct 10, 2012
*      Author: jnicho
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

#include <object_manipulation_tools/controller_utils/SimulatedController.h>

SimulatedController::SimulatedController()
:_CntrlStatePubTopic(CONTROLLER_STATE_TOPIC_NAME),
 _ControlFeedbackPubTopic(CONTROLLER_FEEDBACK_TOPIC_NAME),
 _JointStatePubTopic(JOINT_STATE_TOPIC_NAME),
 _JointTrajSubsTopic(JOINT_TRAJECTORY_TOPIC_NAME),
 _JointNames(),
 _LastJointState(),
 _ProcessingRequest(false)
{

}

SimulatedController::~SimulatedController()
{

}

void SimulatedController::init()
{
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();

	// finding joint names
	std::string matchFound;
	XmlRpc::XmlRpcValue list;
	_JointNames.clear();
	_LastJointState.name.clear();
	_LastJointState.position.clear();
	_LastJointState.velocity.clear();
	_LastJointState.effort.clear();

	if(nh.getParam(JOINT_NAMES_PARAM_NAME,list) ||
			nh.getParam(ros::names::parentNamespace(ros::this_node::getName()) + "/" +JOINT_NAMES_PARAM_NAME,list))
	{
		if(list.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_INFO_STREAM(nodeName + ": Found joints under parameter " + JOINT_NAMES_PARAM_NAME);
			//BOOST_FOREACH(XmlRpc::XmlRpcValue val,list.)
			for(int i = 0; i < list.size(); i++)
			{
				XmlRpc::XmlRpcValue val = list[i];
				if(val.getType() == XmlRpc::XmlRpcValue::TypeString)
				{
					std::string name = static_cast<std::string>(val);
					_JointNames.push_back(name);
					_LastJointState.name.push_back(name);
					_LastJointState.position.push_back(0.0f);
					_LastJointState.velocity.push_back(0.0f);
					_LastJointState.effort.push_back(0.0f);
				}
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM(nodeName + ": Could not retrieve parameter " + JOINT_NAMES_PARAM_NAME);
	}

	sensor_msgs::JointState st = _LastJointState;
	updateLastStateFeedbackMessages(st);

	_JointTrajSubscriber = nh.subscribe(_JointTrajSubsTopic,1,&SimulatedController::callbackJointTrajectory,this);
	_ControllerStatePublisher = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(_CntrlStatePubTopic,
			1);
	_ControllerFeedbackPublisher = nh.advertise<control_msgs::FollowJointTrajectoryFeedback>(_ControlFeedbackPubTopic,1);
	_JointStatePublisher = nh.advertise<sensor_msgs::JointState>(_JointStatePubTopic,1);
	_StatusUpdateTimer = nh.createTimer(ros::Duration(1.0f),&SimulatedController::broadcastState,this);
	ros::spin();
}

void SimulatedController::broadcastState(const ros::TimerEvent &evnt)
{
	//pr2_controllers_msgs::JointTrajectoryControllerState state;
	if(_ProcessingRequest)
	{
		return;
	}
	else
	{
		_LastControllerJointState.header.stamp = _LastControllerTrajState.header.stamp =  ros::Time::now();
		_LastJointState.header.stamp = ros::Time::now();
		_ControllerStatePublisher.publish(_LastControllerJointState);
		_ControllerFeedbackPublisher.publish(_LastControllerTrajState);
		_JointStatePublisher.publish(_LastJointState);
	}
}

void SimulatedController::callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
	std::string nodeName = ros::this_node::getName();

	if(msg->points.empty())
	{
		_ProcessingRequest = false;
		return;
	}
	else
	{
		_ProcessingRequest = true;
	}

	ROS_INFO_STREAM(nodeName<<": Starting joint trajectory execution, "
			<<msg->points.size()<<" points in requested trajectory");

	// the messages below contain the same information, but the are expected by different joint trajectory action servers
	pr2_controllers_msgs::JointTrajectoryControllerState state;
	control_msgs::FollowJointTrajectoryFeedback feedback;

	trajectory_msgs::JointTrajectoryPoint error;
	state.joint_names = feedback.joint_names = _JointNames;
	error.positions = std::vector<double>(state.joint_names.size(),0.0f);
	state.error = feedback.error = error;

	sensor_msgs::JointState jointState;
	jointState.name = state.joint_names;

	//jointState.effort = std::vector<double>(state.joint_names.size(),0.0f);

	// publishing controller and joint states
	ros::Duration duration(0.0f);
	const ros::Duration printInterval(2.0f);
	ros::Time printStartTime = ros::Time::now();
	int counter = 0;
	BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point,msg->points)
	{
		//trajectory_msgs::JointTrajectoryPoint point = msg->points[i];
		state.desired = feedback.desired = point;
		state.actual = feedback.actual = point;
		jointState.position = state.desired.positions;
		jointState.velocity = state.desired.velocities;

		(point.time_from_start - duration).sleep();
		duration = point.time_from_start;

		state.header.stamp = feedback.header.stamp = ros::Time::now();
		jointState.header.stamp = ros::Time::now();

		_ControllerStatePublisher.publish(state);
		_ControllerFeedbackPublisher.publish(feedback);
		_JointStatePublisher.publish(jointState);

		if(ros::Time::now() - printStartTime > printInterval)
		{
			ROS_INFO_STREAM(nodeName<<": Executed "<<
					counter<<" joint trajectory points after "<<point.time_from_start.sec<<" seconds");
			printStartTime = ros::Time::now();
		}
		counter++;
	}

	ROS_INFO_STREAM(nodeName<<": Finished joint trajectory execution, ");

	updateLastStateFeedbackMessages(jointState);
	//_LastJointState = jointState;
	_ProcessingRequest = false;
}

void SimulatedController::updateLastStateFeedbackMessages(const sensor_msgs::JointState &st)
{
	_LastJointState = st;

	trajectory_msgs::JointTrajectoryPoint error;
	error.positions = std::vector<double>(st.name.size(),0.0f);
	error.velocities = std::vector<double>(st.name.size(),0.0f);

	_LastControllerTrajState.joint_names = st.name;
	_LastControllerTrajState.actual.positions = _LastControllerTrajState.desired.positions = st.position;
	_LastControllerTrajState.actual.velocities = _LastControllerTrajState.desired.velocities = st.velocity;
	_LastControllerTrajState.actual.time_from_start = _LastControllerTrajState.desired.time_from_start = ros::Duration(0.0f);
	_LastControllerTrajState.error = error;

	_LastControllerJointState.joint_names = st.name;
	_LastControllerJointState.actual.positions = _LastControllerJointState.desired.positions = st.position;
	_LastControllerJointState.actual.velocities = _LastControllerJointState.desired.velocities = st.velocity;
	_LastControllerJointState.actual.time_from_start = _LastControllerJointState.desired.time_from_start = ros::Duration(0.0f);
	_LastControllerJointState.error = error;
}
