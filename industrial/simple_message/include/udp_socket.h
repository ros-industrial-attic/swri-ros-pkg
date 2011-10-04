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


#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include "sys/socket.h"
#include "arpa/inet.h"
#include "string.h"
#include "smpl_msg_connection.h"

#include "unistd.h"
//#include "fcntl.h"

namespace industrial
{
namespace udp_socket
{

class UdpSocket : industrial::smpl_msg_connection::SmplMsgConnection
{
  public:


  UdpSocket();
    ~UdpSocket();

    /**
       * \brief initializes UDP server socket.  Object can either be a client OR
       * a server, NOT BOTH.
       *
       * \param port_num port number (server & client port number must match)
       *
       * \return true on success, false otherwise (socket is invalid)
       */
    bool initServer(int port_num);

    /**
       * \brief initializes UDP client socket.  Object can either be a client OR
       * a server, NOT BOTH.
       *
       * \param buff server address (in string form) xxx.xxx.xxx.xxx
       * \param port_num port number (server & client port number must match)
       *
       * \return true on success, false otherwise (socket is invalid)
       */
    bool initClient(char *buff, int port_num);

    // Virtual
    bool send(industrial::byte_array::ByteArray & buffer);
    bool receive(industrial::byte_array::ByteArray & buffer, size_t num_bytes);

  

  private:
    int sock_handle_;
    sockaddr_in sockaddr_;

    static const int SOCKET_FAIL = -1;
    static const int MAX_BUFFER_SIZE = 1024;

    /**
       * \brief internal data buffer for receiving
       */
      char buffer_[MAX_BUFFER_SIZE + 1];


      // Override
      // receive is overridden because the base class implementation assumed
      // socket data could be read partially.  UDP socket data is lost when
      // only a portion of it is read.  For that reason this receive method
      // reads the entire data stream (assumed to be a single message).
      bool receiveAllMsgs(industrial::simple_message::SimpleMessage & message);

    int getSockHandle() const{ return sock_handle_;}
    void setSockHandle(int sock_handle_){ this->sock_handle_ = sock_handle_;}
};


}//udp_socket
}//industrial

#endif
