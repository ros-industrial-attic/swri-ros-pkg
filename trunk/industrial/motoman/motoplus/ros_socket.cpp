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

#include "ros_socket.h"

using utils::arrayIntToChar;
using utils::arrayCharToInt;

ROSSocket::ROSSocket(LONG port_num)
// Constructor for UDP socket object
// Creates and binds UDP socket.
{
  sock_handle = mpSocket(AF_INET, SOCK_DGRAM, 0);
			
  memset(&server_sockaddr, 0, sizeof(server_sockaddr));
  server_sockaddr.sin_family = AF_INET;
  server_sockaddr.sin_addr.s_addr = INADDR_ANY;
  server_sockaddr.sin_port = mpHtons(port_num);
		
  rc = mpBind(sock_handle, (sockaddr *)&server_sockaddr, sizeof(server_sockaddr));
		
  sizeof_sockaddr = sizeof(client_sockaddr);
  memset(&client_sockaddr, 0, sizeof_sockaddr);
}

ROSSocket::~ROSSocket(void)
// Destructor for UDP socket object
// Closes socket
{
  mpClose(sock_handle);
}

LONG ROSSocket::sendData(CHAR *raw_message, LONG data_length)
// Sends data to socket; returns number of bytes sent or error
{
  return mpSendTo(sock_handle, raw_message, data_length, 0, (sockaddr *)&client_sockaddr, sizeof(client_sockaddr));
}

LONG ROSSocket::sendMessage(LONG send_message[])
// Sends message to socket; returns number of bytes sent or error
{
  CHAR raw_message[44];
  arrayIntToChar(send_message, raw_message, sizeof(raw_message));
  return sendData(raw_message, sizeof(raw_message));
}

LONG ROSSocket::sendMessage(LONG p0, LONG p1, LONG p2, LONG p3, LONG p4, LONG p5, LONG p6, LONG p7, LONG p8, LONG p9, LONG p10)
// Sends message to socket; returns number of bytes sent or error
{
  LONG send_message[11];
  CHAR raw_message[44];
	
  send_message[0] = p0;
  send_message[1] = p1;
  send_message[2] = p2;
  send_message[3] = p3;
  send_message[4] = p4;
  send_message[5] = p5;
  send_message[6] = p6;
  send_message[7] = p7;
  send_message[8] = p8;
  send_message[9] = p9;
  send_message[10] = p10;
	
  arrayIntToChar(send_message, raw_message, sizeof(raw_message));
  return sendData(raw_message, sizeof(raw_message));
}

LONG ROSSocket::recvData(CHAR *raw_message)
// Receives data from socket; returns number of bytes received or error
{
  return mpRecvFrom(sock_handle, raw_message, BUFF_MAX, 0, (sockaddr *)&client_sockaddr, (int *)&sizeof_sockaddr);
}

LONG ROSSocket::recvMessage(LONG recv_message[])
// Receives message from socket; returns number of bytes received or error
{
  LONG result = ERROR;
  CHAR raw_message[44];
  result = recvData(raw_message);
  arrayCharToInt(raw_message, recv_message, sizeof(raw_message));
  return result;
}