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

#ifndef JOINT_TRAJECTORY_INTERFACE_H
#define JOINT_TRAJECTORY_INTERFACE_H

#include <vector>
#include <string>

#include "ros/ros.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace industrial_robot_client
{
namespace joint_trajectory_interface
{

  using industrial::smpl_msg_connection::SmplMsgConnection;
  using industrial::tcp_client::TcpClient;
  using industrial::joint_traj_pt_message::JointTrajPtMessage;

/**
 * \brief Message handler that relays joint trajectories to the robot controller
 */

//* JointTrajectoryInterface
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryInterface
{

public:

 /**
  * \brief Default constructor.
  */
    JointTrajectoryInterface() : default_joint_val_(0.0) {};

    /**
     * \brief Initialize robot connection using default method.
     *
     * \return true on success, false otherwise
     */
    virtual bool init();

    /**
     * \brief Initialize robot connection using specified method.
     *
     * \param connection new robot-connection instance (ALREADY INITIALIZED).
     *
     * \return true on success, false otherwise
     */
    virtual bool init(SmplMsgConnection* connection);

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names);

  virtual ~JointTrajectoryInterface();

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param msg JointTrajectory message from ROS trajectory-planner
   */
  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  /**
   * \brief Begin processing messages and publishing topics.
   */
  virtual void run() { ros::spin(); }

protected:

  /**
   * \brief Send a stop command to the robot
   */
  virtual void trajectoryStop();

  /**
   * \brief Convert ROS trajectory message into stream of JointTrajPtMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] traj ROS JointTrajectory message
   * \param[out] msgs list of JointTrajPtMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<JointTrajPtMessage>* msgs);

  /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pos_in joint positions, in same order as expected for robot-connection.
   * \param[out] pos_out transformed joint positions (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    *pos_out = pos_in;  // by default, no transform is applied
    return true;
  }

  /**
   * \brief Select specific joints for sending to the robot
   *
   * \param[in] ros_joint_names joint names from ROS command
   * \param[in] ros_joint_pos joint positions from ROS command
   * \param[in] rbt_joint_names joint names, in order/count expected by robot connection
   * \param[out] rbt_joint_pos joint positions, matching rbt_joint_names
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<std::string>& ros_joint_names, const std::vector<double>& ros_joint_pos,
                      const std::vector<std::string>& rbt_joint_names, std::vector<double>* rbt_joint_pos);

  /**
   * \brief Reduce the ROS velocity commands (per-joint velocities) to a single scalar for communication to the robot.
   *   For flexibility, the robot command message contains both "velocity" and "duration" fields.  The specific robot
   *   implementation can utilize either or both of these fields, as appropriate.
   */
  virtual bool calc_velocity(const std::vector<double>& ros_velocities, double* rbt_velocity, double* rbt_duration);

  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param messages List of SimpleMessage JointTrajPtMessages to send to robot.
   *
   * \return true on success, false otherwise
   */
  virtual bool send_to_robot(const std::vector<JointTrajPtMessage>& messages)=0;

  TcpClient default_tcp_connection_;

  ros::NodeHandle node_;
  SmplMsgConnection* connection_;
  ros::Subscriber sub_joint_trajectory_; // handle for joint-trajectory topic subscription
  std::vector<std::string> all_joint_names_;
  double default_joint_val_;  // default value to use for "blank-named" joint commands

private:
  static JointTrajPtMessage create_message(int seq, std::vector<double> joint_pos, double velocity, double duration);
};

} //joint_trajectory_interface
} //industrial_robot_client

#endif /* JOINT_TRAJECTORY_INTERFACE_H */
