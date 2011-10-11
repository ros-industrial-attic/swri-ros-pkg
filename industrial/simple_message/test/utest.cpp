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


#include "simple_message.h"
#include "byte_array.h"
#include "shared_types.h"
#include "smpl_msg_connection.h"
#include "udp_socket.h"
#include "tcp_socket.h"
#include "ping_message.h"
#include "ping_handler.h"
#include "message_manager.h"

#include <gtest/gtest.h>

using namespace industrial::simple_message;
using namespace industrial::byte_array;
using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::udp_socket;
using namespace industrial::tcp_socket;
using namespace industrial::ping_message;
using namespace industrial::ping_handler;
using namespace industrial::message_manager;

TEST(ByteArraySuite, init)
{

  const shared_int SIZE = 100;
  const shared_int TOO_BIG = 5000;

  ByteArray bytes;

  char bigBuffer[TOO_BIG];
  char buffer[SIZE];

  // Valid byte arrays
  EXPECT_TRUE(bytes.init(&buffer[0], SIZE));
  EXPECT_EQ(bytes.getBufferSize(), SIZE);

  // Invalid init (too big)
  // Invalid buffers
  EXPECT_FALSE(bytes.init(&bigBuffer[0], TOO_BIG));
}

TEST(ByteArraySuite, loading)
{
  const shared_int SIZE = 100;
  char buffer[SIZE];

  ByteArray bytes;
  ByteArray empty;

  ASSERT_TRUE(bytes.init(&buffer[0], SIZE));

  shared_bool bIN = true, bOUT = false;
  shared_int iIN = 999, iOUT = 0;
  shared_real rIN = 9999.9999, rOUT = 0;

  // Boolean loading
  EXPECT_TRUE(bytes.load(bIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_bool));
  EXPECT_TRUE(bytes.unload(bOUT));
  EXPECT_EQ(bytes.getBufferSize(), SIZE);
  EXPECT_EQ(bOUT, bIN);

  // Integer loading
  EXPECT_TRUE(bytes.load(iIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_int));
  EXPECT_TRUE(bytes.unload(iOUT));
  EXPECT_EQ(bytes.getBufferSize(), SIZE);
  EXPECT_EQ(iOUT, iIN);

  // Real loading
  EXPECT_TRUE(bytes.load(rIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_real));
  EXPECT_TRUE(bytes.unload(rOUT));
  EXPECT_EQ(bytes.getBufferSize(), SIZE);
  EXPECT_EQ(rOUT, rIN);

  // Unloading a single member (down to an empty buffer size)
  EXPECT_TRUE(empty.load(bIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_bool));
    EXPECT_TRUE(empty.unload(bOUT));
    EXPECT_EQ(empty.getBufferSize(), 0);
    EXPECT_EQ(bOUT, bIN);
}

TEST(ByteArraySuite, copy)
{

  const shared_int SIZE = 100;
    char buffer[SIZE];

  // Copy
  ByteArray copyFrom;
  ByteArray copyTo;

  EXPECT_TRUE(copyFrom.init(&buffer[0], SIZE));
  EXPECT_TRUE(copyTo.load(copyFrom));
  EXPECT_EQ(copyTo.getBufferSize(), SIZE);
  EXPECT_TRUE(copyTo.load(copyFrom));
  EXPECT_EQ(copyTo.getBufferSize(), 2*SIZE);

  //Copy too large
  EXPECT_FALSE(copyTo.load(copyFrom));
  EXPECT_EQ(copyTo.getBufferSize(), 2*SIZE);
}



TEST(SimpleMessageSuite, init)
{
  SimpleMessage msg;
  ByteArray bytes;

  // Valid messages
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::TOPIC, ReplyTypes::INVALID, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::INVALID, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REPLY,ReplyTypes::SUCCESS, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REPLY,ReplyTypes::FAILURE, bytes));

  // Unused command
  EXPECT_FALSE(msg.init(StandardMsgTypes::INVALID, CommTypes::INVALID,ReplyTypes::INVALID, bytes));

  // Service request with a reply
  EXPECT_FALSE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::SUCCESS, bytes));
  EXPECT_FALSE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::FAILURE, bytes));
}

TEST(PingMessageSuite, init)
{
  PingMessage ping;
  SimpleMessage msg;

  EXPECT_FALSE(ping.init(msg));
  ping.init();
  EXPECT_EQ(StandardMsgTypes::PING, ping.getMsgType());

  ping = PingMessage();
  ASSERT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,
                    ReplyTypes::INVALID));
  EXPECT_TRUE(ping.init(msg));
  EXPECT_EQ(StandardMsgTypes::PING, ping.getMsgType());
}


TEST(PingMessageSuite, toMessage)
{
  PingMessage ping;
  SimpleMessage msg;

  ping.init();

  ASSERT_TRUE(ping.toReply(msg));
  EXPECT_EQ(StandardMsgTypes::PING, msg.getMessageType());
  EXPECT_EQ(CommTypes::SERVICE_REPLY, msg.getCommType());
  EXPECT_EQ(ReplyTypes::SUCCESS, msg.getReplyCode());

  ASSERT_TRUE(ping.toRequest(msg));
  EXPECT_EQ(StandardMsgTypes::PING, msg.getMessageType());
  EXPECT_EQ(CommTypes::SERVICE_REQUEST, msg.getCommType());
  EXPECT_EQ(ReplyTypes::INVALID, msg.getReplyCode());

  EXPECT_FALSE(ping.toTopic(msg));

}

TEST(PingHandlerSuite, init)
{
  PingHandler handler;
  UdpSocket udp;

  ASSERT_TRUE(handler.init(&udp));
  EXPECT_EQ(StandardMsgTypes::PING,handler.getMsgType());

  EXPECT_FALSE(handler.init(NULL));

}

TEST(MessageManagerSuite, init)
{
  MessageManager manager;
  UdpSocket udp;

  EXPECT_TRUE(manager.init(&udp));
  EXPECT_FALSE(manager.init(NULL));

}

TEST(MessageManagerSuite, addHandler)
{
  MessageManager manager;
  UdpSocket udp;
  PingHandler handler;

  EXPECT_EQ(0, manager.getNumHandlers());

  ASSERT_TRUE(manager.init(&udp));
  EXPECT_EQ(1, manager.getNumHandlers());
  EXPECT_FALSE(manager.add(NULL));

  ASSERT_TRUE(handler.init(&udp));
  EXPECT_FALSE(manager.add(&handler));
}


TEST(MessageManagerSuite, ping)
{
  const int portNumber = 11000;
  char ipAddr[] = "127.0.0.1";

  UdpSocket udpClient, udpServer;
  TcpSocket tcpClient, tcpServer;
  SimpleMessage pingRequest, pingReply;
  MessageManager udpManager, tcpManager;

  ASSERT_TRUE(pingRequest.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,
                    ReplyTypes::INVALID));

  ASSERT_TRUE(udpServer.initServer(portNumber));
  ASSERT_TRUE(udpClient.initClient(&ipAddr[0], portNumber));

  ASSERT_TRUE(udpManager.init(&udpServer));
  EXPECT_TRUE(udpClient.sendMsg(pingRequest));

  ASSERT_TRUE(tcpServer.initServer(portNumber));
  ASSERT_TRUE(tcpClient.initClient(&ipAddr[0], portNumber));

  ASSERT_TRUE(tcpManager.init(&tcpServer));
  EXPECT_FALSE(tcpClient.sendMsg(pingRequest)); // NOT CONNECTED YET!


}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}




