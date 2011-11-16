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

#ifdef ROS
#include "socket/tcp_socket.h"
#endif

#ifdef MOTOPLUS
#include "tcp_socket.h"
#endif

#include "log_wrapper.h"
#include "simple_message.h"
#include "shared_types.h"

using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial
{
namespace tcp_socket
{

TcpSocket::TcpSocket()
// Constructor for UDP socket object
{
  this->setSockHandle(this->SOCKET_FAIL);
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
  this->setConnected(false);

}

TcpSocket::~TcpSocket()
// Destructor for UDP socket object
// Closes socket
{
  LOG_DEBUG("Destructing TCPSocket");
  CLOSE(this->getSockHandle());
}

bool TcpSocket::sendBytes(ByteArray & buffer)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;

  if (this->isConnected())
  {
    // Nothing restricts the ByteArray from being larger than the what the socket
    // can handle.
    if (this->MAX_BUFFER_SIZE > (int)buffer.getBufferSize())
    {
      rc = SEND(this->getSockHandle(), buffer.getRawDataPtr(), buffer.getBufferSize(), 0);

      if (this->SOCKET_FAIL != rc)
      {
        rtn = true;
      }
      else
      {
        rtn = false;
        LOG_ERROR("Socket sendBytes failed, rc: %d", rc);
      }
    }
    else
    {
      LOG_ERROR("Buffer size: %u, is greater than max socket size: %u", buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
      rtn = false;
    }

  }
  else
  {
    rtn = false;
    LOG_WARN("Not connected, bytes not sent");
  }

  if (!rtn)
    {
      this->setConnected(false);
    }

  return rtn;
}

bool TcpSocket::receiveBytes(ByteArray & buffer, industrial::shared_types::shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;

  // Reset the buffer (this is not required since the buffer length should
  // ensure that we don't read any of the garbage that may be left over from
  // a previous read), but it is good practice.

  memset(&this->buffer_, 0, sizeof(this->buffer_));

  // Doing a sanity check to determine if the byte array buffer is larger than
  // what can be sent in the socket.  This should not happen and might be indicative
  // of some code synchronization issues between the client and server base.
  if (this->MAX_BUFFER_SIZE < (int)buffer.getMaxBufferSize())
  {
    LOG_WARN("Socket buffer max size: %u, is larger than byte array buffer: %u",
             this->MAX_BUFFER_SIZE, buffer.getMaxBufferSize());
  }
  if (this->isConnected())
  {
    rc = RECV(this->getSockHandle(), &this->buffer_[0], num_bytes, 0);

    if (this->SOCKET_FAIL != rc)
    {
      if (rc > 0)
      {
        LOG_DEBUG("Byte array receive, bytes read: %u", rc);
        buffer.init(&this->buffer_[0], rc);
        rtn = true;
      }
      else
      {
        LOG_WARN("Recieved zero bytes: %u", rc);
        rtn = false;
      }
    }
    else
    {
      LOG_ERROR("Socket receive failed, rc: %d", rc);
      LOG_ERROR("Socket errno: %d", errno);
      rtn = false;
    }
  }
  else
  {
    rtn = false;
    LOG_WARN("Not connected, bytes not sent");
  }

  if (!rtn)
  {
    this->setConnected(false);
  }
  return rtn;
}

} //tcp_socket
} //industrial

