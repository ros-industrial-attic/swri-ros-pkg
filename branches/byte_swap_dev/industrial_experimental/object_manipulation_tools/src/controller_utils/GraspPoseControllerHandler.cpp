/*
 * GraspPoseControllerHandler.cpp
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

#include <object_manipulation_tools/controller_utils/GraspPoseControllerHandler.h>

GraspPoseControllerHandler::GraspPoseControllerHandler(const std::string& group_name,const std::string& controller_name)
:TrajectoryControllerHandler(group_name, controller_name),
 grasp_posture_execution_action_client_(controller_name, true)
{
	while(ros::ok() && !grasp_posture_execution_action_client_.waitForServer(ros::Duration(5.0)))
	{
	  ROS_INFO_STREAM("Waiting for the grasp_posture_execution action for group " << group_name << " on the topic " << controller_name << " to come up");
	}
}

bool GraspPoseControllerHandler::executeTrajectory(
		const trajectory_msgs::JointTrajectory& trajectory,
	    boost::shared_ptr<trajectory_execution_monitor::TrajectoryRecorder>& recorder,
	    const trajectory_execution_monitor::TrajectoryFinishedCallbackFunction& traj_callback)
{

	recorder_ = recorder;
	trajectory_finished_callback_ = traj_callback;

	initializeRecordedTrajectory(trajectory);

	object_manipulation_msgs::GraspHandPostureExecutionGoal goal;
	if(trajectory.points[0].positions[0] == 0.0)
	{
		ROS_INFO_STREAM("Should be commanding grasp");
		goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
	}
	else
	{
		ROS_INFO_STREAM("Should be commanding release");
		goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
	}

	grasp_posture_execution_action_client_.sendGoal(goal,
												  boost::bind(&GraspPoseControllerHandler::controllerDoneCallback, this, _1, _2),
												  boost::bind(&GraspPoseControllerHandler::controllerActiveCallback, this),
												  boost::bind(&GraspPoseControllerHandler::controllerFeedbackCallback, this, _1));
	recorder_->registerCallback(group_controller_combo_name_,
							  boost::bind(&GraspPoseControllerHandler::addNewStateToRecordedTrajectory, this, _1, _2, _3));
	return true;
}

void GraspPoseControllerHandler::cancelExecution()
{

}

void GraspPoseControllerHandler::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
		const object_manipulation_msgs::GraspHandPostureExecutionResultConstPtr& result)
{
	recorder_->deregisterCallback(group_controller_combo_name_);
	ROS_INFO_STREAM("Controller is done with state " << (state == actionlib::SimpleClientGoalState::SUCCEEDED));
	done();
}

void GraspPoseControllerHandler::controllerActiveCallback()
{
	ROS_DEBUG_STREAM("Controller went active");
}

void GraspPoseControllerHandler::controllerFeedbackCallback(const object_manipulation_msgs::GraspHandPostureExecutionFeedbackConstPtr& feedback)
{
	ROS_INFO_STREAM("Got feedback");
}

