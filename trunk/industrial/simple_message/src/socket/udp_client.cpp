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

#include "udp_client.h"
#include "log_wrapper.h"

namespace industrial
{
namespace udp_client
{

UdpClient::UdpClient()
{
}

UdpClient::~UdpClient()
{
}

bool UdpClient::init(char *buff, int port_num)
{

  int rc;
  bool rtn;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = SOCKET(AF_INET, SOCK_DGRAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);

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

} //udp_client
} //industrial

