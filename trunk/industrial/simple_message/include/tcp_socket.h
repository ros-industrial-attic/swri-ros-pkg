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

#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#ifdef ROS
#include "sys/socket.h"
#include "arpa/inet.h"
#include "string.h"
#include "unistd.h"
#endif

#ifdef MOTOPLUSE
#include "motoPlus.h"
#endif

#include "simple_socket.h"
#include "shared_types.h"
#include "smpl_msg_connection.h"

namespace industrial
{
namespace tcp_socket
{

class TcpSocket : public industrial::simple_socket::SimpleSocket
{
public:

  TcpSocket();
  ~TcpSocket();

  /**
   * \brief initializes TCP server socket.  Object can either be a client OR
   * a server, NOT BOTH.
   *
   * \param port_num port number (server & client port number must match)
   *
   * \return true on success, false otherwise (socket is invalid)
   */
  bool initServer(int port_num);

  /**
   * \brief initializes TCP client socket.  Object can either be a client OR
   * a server, NOT BOTH.
   *
   * \param buff server address (in string form) xxx.xxx.xxx.xxx
   * \param port_num port number (server & client port number must match)
   *
   * \return true on success, false otherwise (socket is invalid)
   */
  bool initClient(char *buff, int port_num);

  /**
   * \brief connects a client to the server (this function should only be
   * called if a client socket was initialized.
   *
   * \return true on success, false otherwise
   */
  bool connectToServer();

  /**
   * \brief listens for a client connection (this function should only be
   * called if a server socket was initialized.
   *
   * \return true on success, false otherwise
   */
  bool listenForClient();

private:


  // Virtual
bool  sendBytes(industrial::byte_array::ByteArray & buffer);
  bool receiveBytes(industrial::byte_array::ByteArray & buffer,
      industrial::shared_types::shared_int num_bytes);

};

} //tcp_socket
} //industrial

#endif /* TCP_SOCKET_H */
