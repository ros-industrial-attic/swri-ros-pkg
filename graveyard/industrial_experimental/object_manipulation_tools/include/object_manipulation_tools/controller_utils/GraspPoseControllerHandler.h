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


#ifndef _GRASP_POSE_CONTROLLER_HANDLER_H_
#define _GRASP_POSE_CONTROLLER_HANDLER_H_

#include <trajectory_execution_monitor/trajectory_controller_handler.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>

class GraspPoseControllerHandler : public trajectory_execution_monitor::TrajectoryControllerHandler {

public:

  GraspPoseControllerHandler(const std::string& group_name, const std::string& controller_name);

  bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                         boost::shared_ptr<trajectory_execution_monitor::TrajectoryRecorder>& recorder,
                         const trajectory_execution_monitor::TrajectoryFinishedCallbackFunction& traj_callback);

  void cancelExecution();

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const object_manipulation_msgs::GraspHandPostureExecutionResultConstPtr& result);

  void controllerActiveCallback();

  void controllerFeedbackCallback(const object_manipulation_msgs::GraspHandPostureExecutionFeedbackConstPtr& feedback);
    

protected:
  actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> grasp_posture_execution_action_client_;
}; 

#endif

