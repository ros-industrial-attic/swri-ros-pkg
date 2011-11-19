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

#include "joint_message.h"
#include "joint_position.h"
#include "byte_array.h"
#include "log_wrapper.h"

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::joint_position;

namespace industrial
{
namespace joint_message
{

JointMessage::JointMessage(void)
{
  this->setMessageType(StandardMsgTypes::JOINT);
  this->init();
}

JointMessage::~JointMessage(void)
{

}

bool JointMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();

  this->setMessageType(StandardMsgTypes::JOINT);

  if (data.unload(this->joints_))
  {
    if (data.unload(this->sequence_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to unload sequence data");
    }
  }
  else
  {
    LOG_ERROR("Failed to unload joint data");
  }
  return rtn;
}

void JointMessage::init(shared_int seq, JointPosition& joints)
{
  this->setSequence(seq);
  this->joints_.copyFrom(joints);
}

void JointMessage::init()
{
  this->setSequence(0);
  this->joints_.init();
}
bool JointMessage::toRequest(industrial::simple_message::SimpleMessage & msg)
{
  ByteArray data;
  data.load(*this);
  return msg.init(this->getMessageType(), CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID, data);
}

bool JointMessage::toReply(industrial::simple_message::SimpleMessage & msg)
{

  ByteArray data;
  data.load(*this);
  return msg.init(this->getMessageType(), CommTypes::SERVICE_REPLY, ReplyTypes::SUCCESS, data);

}

bool JointMessage::toTopic(industrial::simple_message::SimpleMessage & msg)
{
  ByteArray data;
  LOG_DEBUG("Generating joint message of type topic");
  data.load(*this);
  return msg.init(this->getMessageType(), CommTypes::TOPIC, ReplyTypes::INVALID, data);
}

bool JointMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_DEBUG("Executing joint message load");
  if (buffer->load(this->getSequence()))
  {

    if (buffer->load(this->joints_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to load sequence data");
    }
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load sequence data");
  }
  return rtn;
}

bool JointMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_DEBUG("Executing joint message unload");

  if (buffer->unload(this->joints_))
  {

    if (buffer->unload(this->sequence_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to unload sequence data");
    }
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint data");
  }
  return rtn;
}

}
}

