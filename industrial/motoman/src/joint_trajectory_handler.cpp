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
  this->sub_joint_tranectory_ = this->node_.subscribe("command", 0, &JointTrajectoryHandler::jointTrajectoryCB,
                                                      this);
  this->robot_ = robotConnecton;
  this->currentPoint = 0;
  this->state_ = JointTrajectoryStates::IDLE;
  this->trajectoryHandler_ =
      new boost::thread(boost::bind(&JointTrajectoryHandler::trajectoryHandler, this));
  //this->trajectoryHandler_->join();
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();
}

JointTrajectoryHandler::~JointTrajectoryHandler()
{
  this->sub_joint_tranectory_.shutdown();
  delete this->trajectoryHandler_;
}

void JointTrajectoryHandler::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajctory message");
  this->mutex_.lock();
  ROS_INFO("Processing joint trajctory message (mutex acquired)");
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
    ROS_INFO("Loading trajectory, setting state to starting");
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
  SimpleMessage reply;
  trajectory_msgs::JointTrajectoryPoint pt;
  ROS_INFO("Starting joint trajectory handler state");
  while (ros::ok())
  {
    this->mutex_.lock();

    if (this->robot_->isConnected())
    {
      
      //TODO: These variables should be moved outside of this loop
      //so that we aren't contantly reinitializing them.
      JointMessage jMsg;
      SimpleMessage msg;
      SimpleMessage reply;
      trajectory_msgs::JointTrajectoryPoint pt;
      switch (this->state_)
      {
        case JointTrajectoryStates::IDLE:
          ros::Duration(1).sleep();
          break;

        case JointTrajectoryStates::STARTING:
          ROS_INFO("Joint trajectory handler: entering starting state");
          this->currentPoint = 0;
          this->state_ = JointTrajectoryStates::STREAMING;
          break;

        case JointTrajectoryStates::STREAMING:
          if (this->currentPoint < this->current_traj_.points.size())
          {
	    ROS_INFO("Streaming joints point[%d]", this->currentPoint);
            pt = this->current_traj_.points[this->currentPoint];
            
            jMsg.setSequence(this->currentPoint);

            for (int i = 0; i < this->current_traj_.joint_names.size(); i++)
            {
              jMsg.getJoints().setJoint(i, pt.positions[i]);
            }
            
            jMsg.toRequest(msg);
            ROS_DEBUG("Sending joint point");
            if (this->robot_->sendAndReceiveMsg(msg, reply))
	    //Trying async messages to see how well it works.
	    //if (this->robot_->sendMsg(msg))
            {
              ROS_INFO("Point[%d] sent to controller", this->currentPoint);
              this->currentPoint++;
            }
            else
            {
	      ROS_WARN("Failed sent joint point, will try again");
            }
          }
          else
          {
            ROS_INFO("Trajectory streaming complete, not sending end command, queue remains active.");
            this->state_ = JointTrajectoryStates::IDLE;
            
	  /*  
	      TODO: COMMENTING OUT THE STOP TRAJECTORY FOR NOW.  When the stop
              trajectory signal is caught by the controller it immediately
              stops instead of waiting until it reaches it's goal.  For now
              we will ingnore this.  This shouldn't cause an issue until the
              buffer indexes roll over.
            ROS_INFO("Trajectory streaming complete, sending end command");
	        jMsg.setSequence(SpecialSeqValues::END_TRAJECTORY);
            jMsg.toRequest(msg);
            ROS_DEBUG("Sending end trajectory point");
            if (this->robot_->sendAndReceiveMsg(msg, reply))
            {
              this->state_ = JointTrajectoryStates::IDLE;
            }
            else
            {
	      ROS_WARN("Failed sent joint point, will try again");
            }
            */
          }
          break;

        case JointTrajectoryStates::STOPPING:
          ROS_INFO("Joint trajectory handler: entering stopping state");
          jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
          jMsg.toRequest(msg);
          ROS_DEBUG("Sending stop command");
          this->robot_->sendAndReceiveMsg(msg, reply);
          ROS_DEBUG("Stop command sent, entring idle mode");
          this->state_ = JointTrajectoryStates::IDLE;
          break;

        default:
          ROS_ERROR("Joint trajectory handler: unknown state");
          this->state_ = JointTrajectoryStates::IDLE;
          break;
      }

    }
    else
    {
      ROS_INFO("Connecting to robot motion server");
      this->robot_->makeConnect();
    }

    this->mutex_.unlock();
    ros::Duration(0.005).sleep();
  }

  ROS_WARN("Exiting trajectory handler thread");
}

} //joint_trajectory_handler
} //motoman

