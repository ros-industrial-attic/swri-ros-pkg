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

#ifndef __ros_socket_h
#define __ros_socket_h

#include "motoPlus.h"
#include "definitions.h"
#include "utils.h"

class ROSSocket
// Class for UDP socket object, used to communicate with ROS interface
{
	public:
		ROSSocket(LONG port_num);
		~ROSSocket(void);
		LONG sendData(CHAR *raw_message, LONG data_length);
		LONG sendMessage(LONG send_message[]);
		LONG sendMessage(LONG p0, LONG p1, LONG p2, LONG p3, LONG p4, LONG p5, LONG p6, LONG p7, LONG p8, LONG p9, LONG p10);
		LONG recvData(CHAR *raw_message);
		LONG recvMessage(LONG recv_message[]);
	
	protected:
		LONG sock_handle;
		sockaddr_in server_sockaddr, client_sockaddr;
		LONG rc, bytes_recv, bytes_send, sizeof_sockaddr;
		ULONG port_num;
};

#endif