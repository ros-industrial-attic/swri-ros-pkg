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

#ifndef __p_var_q_h
#define __p_var_q_h

#include "motoPlus.h"
#include "definitions.h"
#include "ros_socket.h"
#include "utils.h"

class PVarQ
// Holds data and functions for executing position variable queue motion
{
  public:
    PVarQ(ROSSocket* sock, bool* motion_allowed);
    ~PVarQ(void);
	LONG addPointPVQ(LONG *message);
		
  protected:
    // Functions
	void startJob(void);
		
	// Servo power variables
	MP_SERVO_POWER_RSP_DATA servo_power_info;
	MP_SERVO_POWER_SEND_DATA servo_power_on;
	MP_STD_RSP_DATA servo_power_error;
		
	// Command point variables
	MP_POSVAR_DATA start_point; // Point to be added to starting queue
	LONG start_counter; // Keeps track of how many intitial points have been added
		
	// Joint speed variable
	MP_VAR_DATA joint_speed;
		
	// Next point variable
	MP_POSVAR_DATA next_point;
		
	// Job variables
	MP_START_JOB_SEND_DATA job_data;
	MP_STD_RSP_DATA job_error;
		
	// Hold variables
	MP_HOLD_SEND_DATA hold_data;
	MP_STD_RSP_DATA hold_error;
		
	// Declarations necessary for loop
	MP_TASK_SEND_DATA task_data;
	MP_CUR_JOB_RSP_DATA cur_job_data;
	USHORT cur_iter, next_iter; // Current iteration; next iteration at which to add point
		
	// Socket
	ROSSocket* sock;
		
	// Motion allowed flag
	bool* motion_allowed;
};

#endif