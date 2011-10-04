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

// ROS-side UDP Socket for ROS-MotoPlus communication

#include "moto_socket.h"

using utils::arrayIntToChar;
using utils::arrayCharToInt;

MotoSocket::MotoSocket(char* buff, unsigned short port_num)
// Constructor for UDP socket object
// Creates and binds UDP socket
{
  timeval tv;
  tv.tv_sec = 10; // timeout in seconds
  tv.tv_usec = 0;
  sock_handle = -1;
  int rc;

  sock_handle = socket(AF_INET, SOCK_DGRAM, 0);

  memset(&server_sockaddr, 0, sizeof(server_sockaddr));
  server_sockaddr.sin_family = AF_INET;
  serveraddr = inet_addr(buff);
  server_sockaddr.sin_addr.s_addr = serveraddr;
  server_sockaddr.sin_port = htons(port_num);

  sizeof_sockaddr = sizeof(server_sockaddr);

  //setsockopt(sock_handle, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  //setsockopt(sock_handle, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  fcntl(sock_handle, F_SETFL, O_NONBLOCK);
  
  memset(&client_sockaddr, 0, sizeof(client_sockaddr));
  client_sockaddr.sin_family = AF_INET;
  client_sockaddr.sin_addr.s_addr = INADDR_ANY;
  client_sockaddr.sin_port = htons(port_num);

  rc = bind(sock_handle, (sockaddr *)&client_sockaddr, sizeof(client_sockaddr));
}

MotoSocket::~MotoSocket()
// Destructor for UDP socket object
// Closes socket
{
  close(sock_handle);
}

int MotoSocket::sendData(char* buff, int data_length, bool is_blocking)
// Sends data to socket; returns number of bytes sent or error
{
  int result = -1;
  if (is_blocking)
  {
    while (ros::ok()) // Allow node to be killed by ROS while blocking
    {
      result = sendto(sock_handle, buff, data_length, 0, (sockaddr *)&server_sockaddr, sizeof(server_sockaddr));
      if (result >= 0)
        break;
    }
  }
  else
    result = sendto(sock_handle, buff, data_length, 0, (sockaddr *)&server_sockaddr, sizeof(server_sockaddr));
  return result;
}

int MotoSocket::sendMessage(int send_message[], bool is_blocking)
// Sends message to socket; returns number of bytes sent or error
{
  char raw_message[44];
  arrayIntToChar(send_message, raw_message, sizeof(raw_message));
  return sendData(raw_message, sizeof(raw_message), is_blocking);
}

int MotoSocket::sendMessage(int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8, int p9, int p10, bool is_blocking)
// Sends message to socket; returns number of bytes sent or error
{
  int send_message[11];
  char raw_message[44];

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
  return sendData(raw_message, sizeof(raw_message), is_blocking);
}

int MotoSocket::recvData(char* buff, bool is_blocking)
// Receives data from socket; returns number of bytes received or error
{
  int result = -1;
  if (is_blocking)
  {
    while (ros::ok()) // Allow node to be killed by ROS while blocking
    {
      result = recvfrom(sock_handle, buff, 1024, 0, (sockaddr *)&server_sockaddr, &sizeof_sockaddr);
      if (result >= 0)
        break;
    }
  }
  else
    result = recvfrom(sock_handle, buff, 1024, 0, (sockaddr *)&server_sockaddr, &sizeof_sockaddr);
  return result;
}

int MotoSocket::recvMessage(int recv_message[], bool is_blocking)
// Receives message from socket; returns number of bytes received or error
{
  int result = -1;
  char raw_message[44];
  result = recvData(raw_message, is_blocking);
  if (result > 0)
    arrayCharToInt(raw_message, recv_message, sizeof(raw_message));
  return result;
}
