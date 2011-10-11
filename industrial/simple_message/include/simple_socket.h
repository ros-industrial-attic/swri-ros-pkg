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

#ifndef SIMPLE_SOCKET_H
#define SIMPLE_SOCKET_H

#ifdef ROS

#include "sys/socket.h"
#include "arpa/inet.h"
#include "string.h"
#include "unistd.h"
#include "netinet/tcp.h"
#include "errno.h"

#define SOCKET(domain, type, protocol) socket(domain, type, protocol)
#define BIND(sockfd, addr, addrlen) bind(sockfd, addr, addrlen)
#define SET_NO_DELAY(sockfd, val) setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val))
#define LISTEN(sockfd, n) listen(sockfd, n)
#define CONNECT(sockfd, dest_addr ,addrlen) connect(sockfd, dest_addr, addrlen)
#define SEND_TO(sockfd, buf, len, flags, dest_addr, addrlen) sendto(sockfd, buf, len, flags, dest_addr, addrlen)
#define SEND(sockfd, buf, len, flags) send(sockfd, buf, len, flags)
#define RECV_FROM(sockfd, buf, len, flags, src_addr, addrlen) recvfrom(sockfd, buf, len, flags, src_addr, addrlen)
#define RECV(sockfd, buf, len, flags) recv(sockfd, buf, len, flags)
#define CLOSE(fd) close(fd)
#define HTONS(num) htons(num)
#define INET_ADDR(str) inet_addr(str)
#define SOCKLEN_T socklen_t

#endif

#ifdef MOTOPLUS

#include "motoPlus.h"

#define SOCKET(domain, type, protocol) mpSocket(domain, type, protocol)
#define BIND(sockfd, addr, addrlen) mpBind(sockfd, addr, addrlen)
#define SET_NO_DELAY(sockfd) -1 //MOTOPLUS does not allow for setting the "no delay" socket option
#define LISTEN(sockfd, n) mpListen(sockfd, n)
#define CONNECT(sockfd, dest_addr ,addrlen) mpConnect(sockfd, dest_addr, addrlen)
#define SEND_TO(sockfd, buf, len, flags, dest_addr, addrlen) mpSendTo(sockfd, buf, len, flags, dest_addr, addrlen)
#define SEND(sockfd, buf, len, flags) mpSend(sockfd, buf, len, flags)
#define RECV_FROM(sockfd, buf, len, flags, src_addr, addrlen) mpRecvFrom(sockfd, buf, len, flags, src_addr, addrlen)
#define RECV(sockfd, buf, len, flags) mpRecv(sockfd, buf, len, flags)
#define CLOSE(fd) mpClose(fd)
#define HTONS(num) mpHtons(num)
#define INET_ADDR(str) mpInetAddr(str)
#define SOCKLEN_T unsigned int

#endif

#include "shared_types.h"
#include "smpl_msg_connection.h"

namespace industrial
{
namespace simple_socket
{

/**
 * \brief Defines socket functions required for a simple connection type.
 */
class SimpleSocket : public industrial::smpl_msg_connection::SmplMsgConnection
{
public:

  /**
   * \brief initializes TCP server socket.  Object can either be a client OR
   * a server, NOT BOTH.
   *
   * \param port_num port number (server & client port number must match)
   *
   * \return true on success, false otherwise (socket is invalid)
   */
  virtual bool initServer(int port_num)=0;

  /**
   * \brief initializes TCP client socket.  Object can either be a client OR
   * a server, NOT BOTH.
   *
   * \param buff server address (in string form) xxx.xxx.xxx.xxx
   * \param port_num port number (server & client port number must match)
   *
   * \return true on success, false otherwise (socket is invalid)
   */
  virtual bool initClient(char *buff, int port_num)=0;

  /**
   * \brief connects a client to the server (this function should only be
   * called if a client socket was initialized.
   *
   * \return true on success, false otherwise
   */
  virtual bool connectToServer()=0;

  /**
   * \brief listens for a client connection (this function should only be
   * called if a server socket was initialized.
   *
   * \return true on success, false otherwise
   */
  virtual bool listenForClient()=0;

protected:
  int sock_handle_;
  sockaddr_in sockaddr_;

  static const int SOCKET_FAIL = -1;
  static const int MAX_BUFFER_SIZE = 1024;

  /**
   * \brief internal data buffer for receiving
   */
  char buffer_[MAX_BUFFER_SIZE + 1];

  int  getSockHandle() const
  { return sock_handle_;}
  void setSockHandle(int sock_handle_)
  { this->sock_handle_ = sock_handle_;}
};

} //simple_socket
} //industrial

#endif /* SIMPLE_SOCKET_H */
