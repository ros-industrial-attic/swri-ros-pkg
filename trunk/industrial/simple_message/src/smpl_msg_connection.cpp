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

#include "smpl_msg_connection.h"
#include "byte_array.h"
#include "log_wrapper.h"

using namespace industrial::simple_message;
using namespace industrial::byte_array;

namespace industrial
{

namespace smpl_msg_connection
{

SmplMsgConnection::SmplMsgConnection()
{
}


SmplMsgConnection::~SmplMsgConnection()
{
}


bool SmplMsgConnection::send(SimpleMessage & message)
{
  bool rtn;
  ByteArray msgData;

  if (message.validateMessage())
  {
    message.toByteArray(msgData);
    rtn = this->send(msgData);
  }
  else
  {
    rtn = false;
    LOG_ERROR("Message validation failed, message not sent");
  }

return rtn;
}


bool SmplMsgConnection::receiveAllMsgs(SimpleMessage & message)
{
  ByteArray lengthBuffer;
  ByteArray msgBuffer;
  int length;

  bool rtn = false;


  rtn = this->receive(lengthBuffer, message.getLengthSize());

  if (rtn)
  {
    rtn = lengthBuffer.unload(length);

    if (rtn)
    {
      rtn = this->receive(msgBuffer, length);

      if (rtn)
      {
        rtn = message.init(msgBuffer);
      }
      else
      {
        LOG_ERROR("Failed to initialize message");
        rtn = false;
      }

    }
    else
    {
      LOG_ERROR("Failed to receive message");
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Failed to receive message length");
    rtn = false;
  }

  return rtn;
}

bool SmplMsgConnection::receive(SimpleMessage & message)
{

  bool rtn = false;

  rtn = this->receiveAllMsgs(message);

  if (rtn)
  {
    if(StandardMsgTypes::PING == message.getMessageType())
    {
      this->send(message);
      rtn = false;
    }
    else
    {
      rtn = true;
    }
  }

  return rtn;
}

bool SmplMsgConnection::sendAndReceive(SimpleMessage & send, SimpleMessage & recv)
{	
  bool rtn = false;
  rtn = this->send(send);
  if (rtn)
  {
    rtn = this->receive(recv);
  }
  else
  {
    rtn = false;
  }

  return rtn;
}


}//smpl_msg_connection
}//industrial
