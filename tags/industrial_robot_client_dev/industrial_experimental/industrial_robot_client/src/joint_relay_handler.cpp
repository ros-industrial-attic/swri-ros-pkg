/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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

#include <algorithm>

#include "industrial_robot_client/joint_relay_handler.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::joint_message;
using namespace industrial::simple_message;
using namespace industrial::shared_types;
using industrial::smpl_msg_connection::SmplMsgConnection;

namespace industrial_robot_client
{
namespace joint_relay_handler
{

bool JointRelayHandler::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->pub_joint_control_state_ =
          this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states",1);

  // save "complete" joint-name list, preserving any blank entries for later use
  this->robot_joint_names_ = joint_names;

  // only publish non-blank joints to ROS
  std::vector<std::string> valid_names;
  for (int i=0; i<joint_names.size(); ++i)
  {
    if (!joint_names[i].empty())
      valid_names.push_back(joint_names[i]);
  }

  this->num_joints_ = valid_names.size();

  this->joint_control_state_.joint_names = valid_names;
  this->joint_control_state_.actual.positions.resize(num_joints_);
  this->joint_control_state_.desired.positions.resize(num_joints_);
  this->joint_control_state_.error.positions.resize(num_joints_);

  this->joint_sensor_state_.name = this->joint_control_state_.joint_names;
  this->joint_sensor_state_.position.resize(num_joints_);
  this->joint_sensor_state_.velocity.resize(num_joints_);
  this->joint_sensor_state_.effort.resize(num_joints_);

  return init((int)StandardMsgTypes::JOINT, connection);
}

bool JointRelayHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{
  bool rtn = false;
  JointMessage joint;
  SimpleMessage msg;

  LOG_INFO("Executing internal CB");

  if (joint.init(in))
  {
    shared_real value;
    int jnt_idx=0;
    for(int msg_idx =0; msg_idx<robot_joint_names_.size(); ++msg_idx)
    {
      if (robot_joint_names_[msg_idx].empty())  // skip over blank-named joints
        continue;

      if (joint.getJoints().getJoint(msg_idx, value))
      {
        this->joint_control_state_.actual.positions[jnt_idx] = value;
        this->joint_sensor_state_.position[jnt_idx] = value;
      }
      else
      {
        this->joint_control_state_.actual.positions[jnt_idx] = 0.0;
        LOG_ERROR("Failed to populate ith(%d) of controller state message", jnt_idx);
      }
      // TODO: For now these values are not populated
      this->joint_control_state_.desired.positions[jnt_idx] = 0.0;
      this->joint_control_state_.error.positions[jnt_idx] = 0.0;

      ++jnt_idx;
    }
    this->joint_control_state_.header.stamp = ros::Time::now();
    this->pub_joint_control_state_.publish(this->joint_control_state_);

    this->joint_sensor_state_.header.stamp = ros::Time::now();
    this->pub_joint_sensor_state_.publish(this->joint_sensor_state_);

    // Reply back to the controller if the sender requested it.
    if (CommTypes::SERVICE_REQUEST == in.getMessageType())
    {
      joint.toReply(msg, ReplyTypes::SUCCESS);
    }
  }
  else
  {
    LOG_ERROR("Failed to initialize joint message");
    rtn = false;
  }

  return rtn;
}



}//namespace joint_relay_handler
}//namespace industrial_robot_client



