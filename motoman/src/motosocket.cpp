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


// UDP Socket for ROS-MotoPlus communication

#include "motosocket.h"

using std::cout;
using std::endl;

MotoSocket::MotoSocket(char *buff, unsigned short portNum)
{
  timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  sockHandle = -1;
  int rc;

  sockHandle = socket(AF_INET, SOCK_DGRAM, 0);

  memset(&serverSockAddr, 0, sizeof(serverSockAddr));
  serverSockAddr.sin_family = AF_INET;
  serverAddr = inet_addr(buff);
  serverSockAddr.sin_addr.s_addr = serverAddr;
  serverSockAddr.sin_port = htons(portNum);

  sizeofSockAddr = sizeof(serverSockAddr);

  setsockopt(sockHandle, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sockHandle, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  
  memset(&clientSockAddr, 0, sizeof(clientSockAddr));
  clientSockAddr.sin_family = AF_INET;
  clientSockAddr.sin_addr.s_addr = INADDR_ANY;
  clientSockAddr.sin_port = htons(portNum);

  rc = bind(sockHandle, (sockaddr *)&clientSockAddr, sizeof(clientSockAddr));
}

MotoSocket::~MotoSocket()
{
  close(sockHandle);
}

int MotoSocket::sendData(char *buff, int dataLength)
{
  return sendto(sockHandle, buff, dataLength, 0, (sockaddr *)&serverSockAddr, sizeof(serverSockAddr));
}

int MotoSocket::receiveData(char *buff)
{
  return recvfrom(sockHandle, buff, 1024, 0, (sockaddr *)&serverSockAddr, &sizeofSockAddr);
}
