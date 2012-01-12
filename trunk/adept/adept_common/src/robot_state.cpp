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

#include "ros/ros.h"
#include "simple_message/socket/simple_socket.h"
#include "simple_message/socket/udp_client.h"
#include "simple_message/socket/tcp_client.h"
#include "adept_common/joint_relay_handler.h"
#include "simple_message/message_manager.h"

using namespace industrial::udp_client;
using namespace industrial::tcp_client;
using namespace industrial::message_manager;
using namespace industrial::simple_socket;
using namespace adept::joint_relay_handler;

int main(int argc, char** argv)
{
  char ip[1024] = "172.21.3.79"; // Robot IP address
  TcpClient connection;
  MessageManager manager;

  ros::init(argc, argv, "state_interface");
  ros::NodeHandle n;

  JointRelayHandler jr_handler(n);

  ROS_INFO("Setting up client");
  connection.init(ip, StandardSocketPorts::STATE);
  connection.makeConnect();

  jr_handler.init(&connection);

  manager.init(&connection);
  manager.add(&jr_handler);

  manager.spin();
}

