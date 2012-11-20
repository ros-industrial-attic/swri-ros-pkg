﻿/*
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

#ifndef JOINT_TRAJECTORY_DOWNLOADER_H
#define JOINT_TRAJECTORY_DOWNLOADER_H

#include "simple_message/smpl_msg_connection.h"
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>

namespace industrial_robot_client
{
namespace joint_trajectory_downloader
{

/**
 * \brief Message handler that downloads joint trajectories to
 * a robot controller that supports the trajectory downloading interface
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryDownloader
{

public:

  JointTrajectoryDownloader();

  /**
   * \brief Constructor
   *
   * \param n ROS node handle (used for subscribing)
   * \param robot_connection message connection (used for publishing (to the robot controller))
   * \param joint_names ordered list of joint names (controller order)
   */
	JointTrajectoryDownloader(ros::NodeHandle &n,
	                          industrial::smpl_msg_connection::SmplMsgConnection* robot_connecton,
	                          std::vector<std::string> &joint_names);

  /**
   * \brief Destructor
   *
   */
  ~JointTrajectoryDownloader();

  /**
   * \brief Joint trajectory callback.  When receiving a joint trajectory, this
   * method formats it for sending to the robot controller.
   *
   * \param msg joint trajectory
   */
  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

private:
  /**
   * \brief Robot connection
   */
  industrial::smpl_msg_connection::SmplMsgConnection* robot_;

  /**
   * \brief Joint trajectory subscriber
   */
  ros::Subscriber sub_joint_trajectory_; //subscribe to "command"

  /**
   * \brief ROS Node
   *
   */
  ros::NodeHandle node_;

  /**
   * \brief Order list of joint names (controller order).  This list is used to
   * check and reorder of trajectories that are sent to the controller.
   *
   */
  std::vector<std::string> joint_names_;

  /**
   * \brief Internal method for sending trajectory stop command to the controller
   */
  void trajectoryStop();

};

} //joint_trajectory_downloader
} //industrial_robot_client

#endif /* JOINT_TRAJECTORY_DOWNLOADER_H */