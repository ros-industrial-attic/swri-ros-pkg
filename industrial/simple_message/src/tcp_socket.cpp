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

#include "tcp_socket.h"
#include "log_wrapper.h"
#include "simple_message.h"

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

}

TcpSocket::~TcpSocket()
// Destructor for UDP socket object
// Closes socket
{
  CLOSE(this->getSockHandle());
}

bool TcpSocket::initServer(int port_num)
{
  int rc;
  bool rtn;
  int err;
  SOCKLEN_T addrSize = 0;
  int disableNodeDelay = 1;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_STREAM - TCP type
   * protocol (0) - System chooses
   */
  rc = SOCKET(AF_INET, SOCK_STREAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);
    LOG_DEBUG("Socket created, rc: %d", rc);
    LOG_DEBUG("Socket handle: %d", this->getSockHandle());

    // The set no delay disables the NAGEL algorithm
    rc = SET_NO_DELAY(this->getSockHandle(), disableNodeDelay);
    err = errno;
    if (this->SOCKET_FAIL == rc)
    {
      LOG_WARN("Failed to set no socket delay, errno: %d, sending data can be delayed by up to 250ms", err);
    }

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INADDR_ANY;
    this->sockaddr_.sin_port = HTONS(port_num);

    addrSize = sizeof(this->sockaddr_);
    rc = BIND(this->getSockHandle(), (sockaddr *)&(this->sockaddr_), addrSize);

    if (this->SOCKET_FAIL != rc)
    {
      LOG_INFO("Server socket successfully initialized");

      rc = LISTEN(this->getSockHandle(), 1);

      if (this->SOCKET_FAIL != rc)
      {
        LOG_INFO("Socket in listen mode");
        rtn = true;
      }
      else
      {
        LOG_ERROR("Failed to set socket to listen");
        rtn = false;
      }
    }
    else
    {
      LOG_ERROR("Failed to bind socket, rc: %d", rc);
      CLOSE(this->getSockHandle());
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

bool TcpSocket::listenForClient()
{
  bool rtn = false;
  int rc = this->SOCKET_FAIL;
  int socket = this->SOCKET_FAIL;

  rc = ACCEPT(this->getSockHandle(), NULL, NULL);

  if (this->SOCKET_FAIL != rc)
  {
    // The accept call above creates a new socket.  The old
    // socket handle is no longer needed, since only a single
    // server-client socket connection is allowed.
    CLOSE(this->getSockHandle());
    this->setSockHandle(rc);
    LOG_INFO("Client socket accepted");
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to accept for client connection");
    rtn = false;
  }

  return rtn;

}

bool TcpSocket::initClient(char *buff, int port_num)
{

  int rc;
  bool rtn;
  int disableNodeDelay = 1;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = SOCKET(AF_INET, SOCK_STREAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);

    // The set no delay disables the NAGEL algorithm
    rc = SET_NO_DELAY(this->getSockHandle(), disableNodeDelay);
    if (this->SOCKET_FAIL == rc)
    {
      LOG_WARN("Failed to set no socket delay, sending data can be delayed by up to 250ms");
    }

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INET_ADDR(buff);
    this->sockaddr_.sin_port = HTONS(port_num);

    rtn = true;

  }
  else
  {
    LOG_ERROR("Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}

bool TcpSocket::connectToServer()
{
  bool rtn = false;
  int rc = this->SOCKET_FAIL;
  SOCKLEN_T addrSize = 0;

  addrSize = sizeof(this->sockaddr_);
  rc = CONNECT(this->getSockHandle(), (sockaddr *)&this->sockaddr_, addrSize);
  if (this->SOCKET_FAIL != rc)
  {
    LOG_INFO("Connected to server");
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to connect to server");
    rtn = false;
  }

  return rtn;

}

bool TcpSocket::sendBytes(ByteArray & buffer)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;

  // Nothing restricts the ByteArray from being larger than the what the socket
  // can handle.
  if (this->MAX_BUFFER_SIZE > buffer.getBufferSize())
  {
    rc = SEND(this->getSockHandle(), buffer.getRawDataPtr(), buffer.getBufferSize(), 0);

    if (this->SOCKET_FAIL != rc)
    {
      rtn = true;
    }
    else
    {
      LOG_ERROR("Socket sendBytes failed, rc: %d", rc);
    }
  }
  else
  {
    LOG_ERROR("Buffer size: %u, is greater than max socket size: %u", buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
    rtn = false;
  }

  return rtn;
}

bool TcpSocket::receiveBytes(ByteArray & buffer, shared_int num_bytes)
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
  if (this->MAX_BUFFER_SIZE < buffer.getMaxBufferSize())
  {
    LOG_WARN("Socket buffer max size: %u, is larger than byte array buffer: %u",
             this->MAX_BUFFER_SIZE, buffer.getMaxBufferSize());
  }

  // On some platforms (motoplus) the RECV function returns zero bytes instead
  // of blocking until num_bytes is read.  The logic below assumes that either
  // zero or num_bytes is read (This may cause some issues).
  do{
    rc = RECV(this->getSockHandle(), &this->buffer_[0], num_bytes, 0);
  }
  while(rc == 0);

  if (this->SOCKET_FAIL != rc)
  {
    LOG_DEBUG("Byte array receive, bytes read: %u", rc);
    buffer.init(&this->buffer_[0], rc);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Socket receive failed, rc: %d", rc);
    LOG_ERROR("Socket errno: %d", errno);
    rtn = false;
  }
  return rtn;
}

} //tcp_socket
} //industrial

