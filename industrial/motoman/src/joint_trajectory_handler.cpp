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

#include "joint_trajectory_handler.h"
#include "joint_message.h"
#include "smpl_msg_connection.h"

using namespace industrial::smpl_msg_connection;
using namespace industrial::joint_message;
using namespace industrial::simple_message;

namespace motoman
{
namespace joint_trajectory_handler
{
JointTrajectoryHandler::JointTrajectoryHandler()
{
}

JointTrajectoryHandler::JointTrajectoryHandler(ros::NodeHandle &n, SmplMsgConnection* robotConnecton) :
    node_(n)
{
  ROS_INFO("Constructor joint trajectory handler node");

  this->mutex_.lock();
  this->sub_joint_tranectory_ = this->node_.subscribe("command", 1000, &JointTrajectoryHandler::jointTrajectoryCB,
                                                      this);
  this->robot_ = robotConnecton;
  this->currentPoint = 0;
  this->state_ = JointTrajectoryStates::IDLE;
  this->trajectoryHandler_ =
      new boost::thread(boost::bind(&JointTrajectoryHandler::trajectoryHandler, this));
  this->trajectoryHandler_->join();
  this->mutex_.unlock();
}

JointTrajectoryHandler::~JointTrajectoryHandler()
{
  this->sub_joint_tranectory_.shutdown();
  delete this->trajectoryHandler_;
}

void JointTrajectoryHandler::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  this->mutex_.lock();
  if (JointTrajectoryStates::IDLE != this->state_)
  {
    if (msg->points.empty())
    {
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    }
    else
    {
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");
    }

    this->state_ = JointTrajectoryStates::STOPPING;
  }
  else
  {
    this->current_traj_ = *msg;
    this->currentPoint = 0;
    this->state_ = JointTrajectoryStates::STARTING;
  }
  this->mutex_.unlock();
}

void JointTrajectoryHandler::trajectoryHandler()
{
  JointMessage jMsg;
  SimpleMessage msg;
  trajectory_msgs::JointTrajectoryPoint pt;
  ROS_INFO("Starting joint trajectory handler state");
  while (ros::ok())
  {

    this->mutex_.lock();
    if (this->robot_->isConnected())
    {

      switch (this->state_)
      {
        case JointTrajectoryStates::IDLE:
          break;

        case JointTrajectoryStates::STARTING:
          ROS_INFO("Joint trajectory handler: entering starting state");
          this->currentPoint = 0;
          this->state_ = JointTrajectoryStates::STREAMING;
          break;

        case JointTrajectoryStates::STREAMING:
          if (this->currentPoint < this->current_traj_.points.size())
          {
            pt = this->current_traj_.points[this->currentPoint];
            for (int i = 0; i < this->current_traj_.joint_names.size(); i++)
            {
              jMsg.getJoints().setJoint(i, pt.positions[i]);
              jMsg.setSequence(this->currentPoint);
              jMsg.toRequest(msg);
            }
            if (this->robot_->sendMsg(msg))
            {
              ROS_INFO("Point[%d] sent to controller", this->currentPoint);
              this->currentPoint++;
            }
          }
          else
          {
            ROS_INFO("Trajectory streaming complete, sending stop command");
            this->state_ = JointTrajectoryStates::STOPPING;
          }
          break;

        case JointTrajectoryStates::STOPPING:
          ROS_INFO("Joint trajectory handler: entering stopping state");
          jMsg.setSequence(-1);
          jMsg.toRequest(msg);
          this->robot_->sendMsg(msg);
          this->state_ = JointTrajectoryStates::IDLE;
          break;

        default:
          ROS_ERROR("Joint trajectory handler: unknown state");
          this->state_ = JointTrajectoryStates::IDLE;
      }

    }
    else
    {
      this->robot_->makeConnect();
    }

    this->mutex_.unlock();
    sleep(10);
  }
}

} //joint_trajectory_handler
} //motoman

