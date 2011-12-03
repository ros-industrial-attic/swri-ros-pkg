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

/*
#include "system.h"

System::System(ROSSocket* sock, bool* motion_allowed)
{
  // Set up socket
  this->sock = sock;

  this->motion_allowed = motion_allowed;
}

System::~System(void)
{
}

LONG System::hold(bool hold_on)
{
  LONG result;

  if (hold_on)
    hold_data.sHold = ON;
  else
    hold_data.sHold = OFF;
  result = mpHold(&hold_data, &hold_error);
  *motion_allowed = false;
  if (result == ERROR)
    return RC_MP_ERROR;
  else
    return RC_SUCCESS;
  //sock->sendMessage(CMD_ACK, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
}

void System::getFBPulse(LONG reply_message[])
// Retrieves feedback position in pulse counts and copies into reply message
{
  pulse_send_data.sCtrlGrp = 0; // Robot 1
  LONG result = mpGetFBPulsePos(&pulse_send_data, &pulse_recv_data);
  reply_message[0] = CMD_GET_FB_PULSE;
  if (result == ERROR) // Send back error
  {
    reply_message[1] = RC_MP_ERROR;
    for (SHORT i = 2; i < 11; i++)
      reply_message[i] = UNUSED;
  }
  else // Send back data
  {
    reply_message[1] = RC_SUCCESS;
    for (SHORT i = 0; i < 8; i++)
      reply_message[i+2] = pulse_recv_data.lPos[i];
    reply_message[10] = UNUSED;
  }
}

void System::getFBSpeed(LONG reply_message[])
// Retrives feedback speed in pulse counts for each axis and copies into reply message
// Units: pulses per second
{
  speed_send_data.sCtrlGrp = 0; // Robot 1
  LONG result = mpGetFBSpeed(&speed_send_data, &speed_recv_data);
  reply_message[0] = CMD_GET_FB_SPEED;
  if (result == ERROR) // Send back error
  {
    reply_message[1] = RC_MP_ERROR;
    for (SHORT i = 2; i < 11; i++)
      reply_message[i] = UNUSED;
  }
  else // Send back data
  {
    reply_message[1] = RC_SUCCESS;
    for (SHORT i = 0; i < 8; i++)
      reply_message[i+2] = speed_recv_data.lSpeed[i];
    reply_message[10] = UNUSED;
  }
}

void System::getTorque(LONG reply_message[])
// Retrieves percentage of maximum current servo torque value for each axis and copies into reply message
// Units: 0.01% of max torque
{
  torque_send_data.sCtrlGrp = 0; // Robot 1
  LONG result = mpGetTorque(&torque_send_data, &torque_recv_data);
  reply_message[0] = CMD_GET_TORQUE;
  if (result == ERROR) // Send back error
  {
    reply_message[1] = RC_MP_ERROR;
    for (SHORT i = 2; i < 11; i++)
      reply_message[i] = UNUSED;
  }
  else // Send back data
  {
    reply_message[1] = RC_SUCCESS;
    for (SHORT i = 0; i < 8; i++)
      reply_message[i+2] = torque_recv_data.lTorquePcnt[i];
    reply_message[10] = UNUSED;
  }
}

*/