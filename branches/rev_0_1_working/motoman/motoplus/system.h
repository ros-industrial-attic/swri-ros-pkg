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

#ifndef __system_h
#define __system_h

#include "motoPlus.h"

/*
class System
{
  public:
    System(ROSSocket* sock, bool* motion_allowed);
    ~System(void);
    LONG hold(bool hold_on);
    void getFBPulse(LONG reply_message[]);
    void getFBSpeed(LONG reply_message[]);
    void getTorque(LONG reply_message[]);
		
  protected:		
	// Hold variables
	MP_HOLD_SEND_DATA hold_data;
	MP_STD_RSP_DATA hold_error;
		
	// Pulse feedback variables
	MP_CTRL_GRP_SEND_DATA pulse_send_data;
	MP_FB_PULSE_POS_RSP_DATA pulse_recv_data;
	
	// Speed feedback variables
	MP_CTRL_GRP_SEND_DATA speed_send_data;
	MP_FB_SPEED_RSP_DATA speed_recv_data;
	
	// Torque variables
	MP_CTRL_GRP_SEND_DATA torque_send_data;
	MP_TORQUE_RSP_DATA torque_recv_data;
	
	//Socket
	ROSSocket *sock;
		
	// Motion allowed flag
	bool* motion_allowed;
};
*/

#endif