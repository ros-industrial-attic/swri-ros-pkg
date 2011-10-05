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
#include "smpl_msg_connection.h"
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

UdpSocket::UdpSocket() : SmplMsgConnection()
// Constructor for UDP socket object
// Creates and binds UDP socket
{
  this->setSockHandle(this->SOCKET_FAIL);
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));

  }

UdpSocket::~UdpSocket()
// Destructor for UDP socket object
// Closes socket
{
  close(this->getSockHandle());
}

bool UdpSocket::initServer(int port_num)
{
  int rc;
  bool rtn;
  socklen_t addrSize = 0;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);
    LOG_DEBUG("Socket created, rc: %d", rc);
    LOG_DEBUG("Socket handle: %d", this->getSockHandle());


    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INADDR_ANY;
    this->sockaddr_.sin_port = htons(port_num);

    // This set the socket to be non-blocking (NOT SURE I WANT THIS) - sme
    //fcntl(sock_handle, F_SETFL, O_NONBLOCK);

    addrSize = sizeof(this->sockaddr_);
    rc = bind(this->getSockHandle(), (sockaddr *)&(this->sockaddr_), addrSize);

    if (this->SOCKET_FAIL != rc)
    {
      rtn = true;
      LOG_INFO("Server socket successfully initialized");
    }
    else
    {
      LOG_ERROR("Failed to bind socket, rc: %d", rc);
      close(this->getSockHandle());
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}

bool UdpSocket::initClient(char *buff, int port_num)
{

  int rc;
  bool rtn;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = inet_addr(buff);
    this->sockaddr_.sin_port = htons(port_num);

    rtn = true;

  }
  else
  {
    LOG_ERROR("Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}



bool UdpSocket::receiveAllMsgs(SimpleMessage & message)
{
  ByteArray msgBuffer;

  bool rtn = false;

  rtn = this->receive(msgBuffer, 0);

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




bool UdpSocket::send(ByteArray & buffer)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;

  // Nothing restricts the ByteArray from being larger than the what the socket
  // can handle.
  if (this->MAX_BUFFER_SIZE > buffer.getBufferSize())
  {
    rc = sendto(this->getSockHandle(), buffer.getRawDataPtr(),
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
    LOG_ERROR("Buffer size: %u, is greater than max socket size: %u",
              buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
    rtn = false;
  }


  return rtn;
}



bool UdpSocket::receive(ByteArray & buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;
  socklen_t addrSize = 0;


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

  rc = recvfrom(this->getSockHandle(), &this->buffer_, this->MAX_BUFFER_SIZE,
                0, (sockaddr *)&this->sockaddr_, (socklen_t*)&addrSize);

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




}//udp_socket
}//industrial
