/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author E. Gil Jones */

#include <robotiq/robotiq_trajectory_controller_handler.h>
#include <pluginlib/class_list_macros.h>

namespace robotiq {

bool RobotIqTrajectoryControllerHandler::initialize(const std::string& group_name,
                                                    const std::string& controller_name,
                                                    const std::string& ns_name) 
{
  ros::NodeHandle nh;
  TrajectoryControllerHandler::initialize(group_name, controller_name, ns_name);
  state_subscriber_ = nh.subscribe(controller_name+"/state", 25, &RobotIqTrajectoryControllerHandler::handStateCallback, this);
  command_publisher_ = nh.advertise<riq_msgs::RIQHandCommand>(controller_name+"/"+ns_name, 1, true);
  return true;
}

  /// \brief Start executing.  Gripper will either open or close.
bool RobotIqTrajectoryControllerHandler::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                                           boost::shared_ptr<trajectory_execution::TrajectoryRecorder>& recorder,
                                                           const trajectory_execution::TrajectoryFinishedCallbackFunction& traj_callback)
{
  //no need for recorder - we'll just get it in from states
  //recorder_ = recorder;
  trajectory_finished_callback_ = traj_callback;
  
  initializeRecordedTrajectory(trajectory);
  
  riq_msgs::RIQHandCommand hand_command;

  std::map<std::string, double> value_map;
  for(unsigned int i = 0; i < trajectory.joint_names.size(); i++) {
    value_map[trajectory.joint_names[i]] = trajectory.points[0].positions[i];
  }

  //need to determine which mode to command
  if(value_map["palm_finger_1_joint"] < -0.05) {
    hand_command.mode = riq_msgs::RIQHandCommand::PINCH;
  } else if(value_map["palm_finger_1_joint"] > 0.10) {
    hand_command.mode = riq_msgs::RIQHandCommand::SPHERIOD;
  } else {
    hand_command.mode = riq_msgs::RIQHandCommand::CYLINDRICAL;
  }

  //need to determine whether to say open or close
  if(fabs(value_map["finger_0_joint_1"]-value_map["finger_1_joint_1"]) < .0001 &&
     fabs(value_map["finger_1_joint_1"]-value_map["finger_2_joint_1"]) < .0001) {
     hand_command.action = riq_msgs::RIQHandCommand::OPEN;
     ROS_INFO_STREAM("All finger values the same so opening");
  } else {
    hand_command.action = riq_msgs::RIQHandCommand::CLOSE;
    ROS_INFO_STREAM("Some finger different so closing");
  }

  command_publisher_.publish(hand_command);

  //recorder_->registerCallback(group_controller_ns_combo_name_, 
  //boost::bind(&Pr2GripperTrajectoryControllerHandler::addNewStateToRecordedTrajectory, this, _1, _2, _3));
  return true;
}

void RobotIqTrajectoryControllerHandler::handStateCallback(const riq_msgs::RIQHandStateConstPtr& state) 
{
  if(!state->operational) {
    ROS_WARN_STREAM("Hand reports not operational");
  }
  if(controller_state_ == trajectory_execution::TrajectoryControllerStates::EXECUTING) {
    if(state->status == riq_msgs::RIQHandState::IN_PROGRESS) {
      //should add to recorded trajectory
      return;
    } else if(state->status == riq_msgs::RIQHandState::FAULTED || state->status == riq_msgs::RIQHandState::ILLEGAL_OR_UNDEFINED) {
      controller_state_ = trajectory_execution::TrajectoryControllerStates::EXECUTION_FAILURE;
    } else if(state->status == riq_msgs::RIQHandState::SUCCESSFUL) {
      controller_state_ = trajectory_execution::TrajectoryControllerStates::SUCCESS;
    }
    done();
  } 
}

}

PLUGINLIB_DECLARE_CLASS(robotiq, RobotIqTrajectoryControllerHandler,
                        robotiq::RobotIqTrajectoryControllerHandler,
                        trajectory_execution::TrajectoryControllerHandler);
