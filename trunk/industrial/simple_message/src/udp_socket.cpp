/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Yaskawa America, Inc.
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
 *       * Neither the name of the Yaskawa America, Inc., nor the names
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

#include "udp_socket.h"
#include "log_wrapper.h"
#include "simple_message.h"

using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial
{
namespace udp_socket
{

UdpSocket::UdpSocket()
// Constructor for UDP socket object
{
  this->setSockHandle(this->SOCKET_FAIL);
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));

}

UdpSocket::~UdpSocket()
// Destructor for UDP socket object
// Closes socket
{
  CLOSE(this->getSockHandle());
}

bool UdpSocket::receiveMsg(SimpleMessage & message)
{
  ByteArray msgBuffer;

  bool rtn = false;

  rtn = this->receiveBytes(msgBuffer, 0);

  if (rtn)
  {
    LOG_DEBUG("Recieve message bytes: %u", msgBuffer.getBufferSize());
    rtn = message.init(msgBuffer);

    if (rtn)
    {
      rtn = true;
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

  return rtn;
}

bool UdpSocket::sendBytes(ByteArray & buffer)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;

  // Nothing restricts the ByteArray from being larger than the what the socket
  // can handle.
  if (this->MAX_BUFFER_SIZE > buffer.getBufferSize())
  {
    rc = SEND_TO(this->getSockHandle(), buffer.getRawDataPtr(),
        buffer.getBufferSize(), 0, (sockaddr *)&this->sockaddr_,
        sizeof(this->sockaddr_));
    if (this->SOCKET_FAIL != rc)
    {
      rtn = true;
    }
    else
    {
      LOG_ERROR("Socket send failed, rc: %d", rc);
    }
  }
  else
  {
    LOG_ERROR("Buffer size: %u, is greater than max socket size: %u", buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
    rtn = false;
  }

  return rtn;
}

bool UdpSocket::receiveBytes(ByteArray & buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;
  SOCKLEN_T addrSize = 0;

  // Reset the buffer (this is not required since the buffer length should
  // ensure that we don't read any of the garbage that may be left over from
  // a previous read), but it is good practice.

  memset(&this->buffer_, 0, sizeof(this->buffer_));

  // Doing a sanity check to determine if the byte array buffer is larger than
  // what can be sent in the socket.  This should not happen and might be indicative
  // of some code synchronization issues between the client and server base.
  if (this->MAX_BUFFER_SIZE < buffer.getMaxBufferSize())
  {
    LOG_WARN("Socket buffer max size: %u, is larger than byte array buffer: %u",
             this->MAX_BUFFER_SIZE, buffer.getMaxBufferSize());
  }

  addrSize = sizeof(this->sockaddr_);

  rc = RECV_FROM(this->getSockHandle(), &this->buffer_[0], this->MAX_BUFFER_SIZE,
      0, (sockaddr *)&this->sockaddr_, &addrSize);

  if (this->SOCKET_FAIL != rc)
  {
    LOG_DEBUG("Byte array receive, bytes read: %u", rc);
    buffer.init(&this->buffer_[0], rc);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Socket receive failed, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}

} //udp_socket
} //industrial

