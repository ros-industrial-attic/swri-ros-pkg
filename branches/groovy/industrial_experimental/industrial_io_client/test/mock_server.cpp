/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage
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

#include <vector>
#include <algorithm>

// Headers from simple_message
#include "simple_message.h"
#include "shared_types.h"
#include "message_manager.h"
#include "socket/tcp_server.h"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <ctype.h>

int g_argc;
char** g_argv;
const unsigned int g_num_vals = 4;

TEST(MessageManagerSuite, tcp)
{
  // Usage: mock_server <port> val1 .... valN
  ASSERT_GE(g_argc, 3);
  int port = atoi(g_argv[1]);
  std::vector<int> values;
  for(int i=2; i<g_argc; i++)
  {
    if(g_argv[i] && isdigit(g_argv[i][0]))
      values.push_back(atoi(g_argv[i]));
  }
  ASSERT_EQ(values.size(), g_num_vals);
  // The ByteArray::unload() calls will pull the values off in reverse
  // order, so we'll reverse the order in which we expect them.
  std::reverse(values.begin(), values.end());

  industrial::tcp_server::TcpServer tcpServer;
  industrial::message_manager::MessageManager tcpManager;

  // Construct server
  ASSERT_TRUE(tcpServer.init(port));

  // Listen for client connection, init manager and start thread
  ASSERT_TRUE(tcpServer.makeConnect());
  ASSERT_TRUE(tcpManager.init(&tcpServer));

  industrial::simple_message::SimpleMessage msg;
  ASSERT_TRUE(tcpServer.receiveMsg(msg));
  ASSERT_EQ(msg.getMessageType(), industrial::simple_message::StandardMsgTypes::IO);
  ASSERT_EQ(msg.getCommType(), industrial::simple_message::CommTypes::TOPIC);
  ASSERT_EQ(msg.getReplyCode(), industrial::simple_message::ReplyTypes::INVALID);
  ASSERT_EQ(msg.getDataLength(), (int)(g_num_vals*4));
  industrial::byte_array::ByteArray buf = msg.getData();
  for(unsigned int i=0; i<g_num_vals; i++)
  {
    int val;
    buf.unload(val);
    ASSERT_EQ(val, values[i]);
  }
}

int
main(int argc, char** argv)
{
  g_argc = argc;
  g_argv = argv;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
