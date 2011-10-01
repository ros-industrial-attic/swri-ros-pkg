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

#include "p_var_q.h"

using utils::arrayIntToChar;
using utils::arrayCharToInt;

PVarQ::PVarQ(ROSSocket* sock, bool* motion_allowed)
// Initializes position variable queue motion
{	
  // Set servo power variable
  servo_power_on.sServoPower = ON;

  // Set up start point variable
  start_point.usType = MP_RESTYPE_VAR_ROBOT;
  
  // Set joint speed variable
  joint_speed.usType = MP_RESTYPE_VAR_I;

  // Initialize counter for starting points
  start_counter = 0;
	
  // Set up next point variable
  next_point.usType = MP_RESTYPE_VAR_ROBOT;
  next_point.ulValue[0] = 0;
	
  // Set up job variable
  job_data.sTaskNo = 0;
  strcpy(job_data.cJobName, JOBNAME);

  // Declarations necessary for loop
  next_iter = 0; // Current iteration and next loop iteration at which to copy data
  task_data.sTaskNo = 0;
	
  // Set up socket
  this->sock = sock;

  // Set up motion allowed flag
  this->motion_allowed = motion_allowed;
}

PVarQ::~PVarQ(void)
// Puts robot on Hold status
{
  //servo_power_on.sServoPower = OFF;
  //mpSetServoPower(&servo_power_on, &servo_power_error);
  // Hold
  hold_data.sHold = ON;
  mpHold(&hold_data, &hold_error);

  // Send ack message
  //this->sock->sendMessage(CMD_ACK, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
}

LONG PVarQ::addPointPVQ(LONG* message)
// Handles addition of new trajectory point. Adds to end of starting queue if it isn't full yet; else adds it to moving queue.
{
  LONG pulse_coords[8];
  LONG velocity;
	
  // Extract position data from message
  for (SHORT i = 0; i < 8; i++)
    pulse_coords[i] = message[i+2];
   
  // Extract velocity from message
  velocity = message[10];
	
  // If starting queue isn't full yet, add point to end of it
  if (start_counter < (QSIZE-1))
  {
    // Position
	start_point.usIndex = start_counter;
	start_point.ulValue[0] = 0;
	for (SHORT i = 0; i < 8; i++)
	{
	  start_point.ulValue[i+2] = pulse_coords[i];
	}
	if (mpPutPosVarData(&start_point, 1) == ERROR)
	  return RC_MP_ERROR;
	
	// Velocity
	joint_speed.usIndex = start_counter;
	joint_speed.ulValue = velocity;
	if (mpPutVarData(&joint_speed, 1) == ERROR)
	  return RC_MP_ERROR;
	
	// Increment starting queue counter
	start_counter++;
  }
  else if (start_counter >= (QSIZE-1))
  {
    // If starting queue has just been filled, start job
	if (start_counter == (QSIZE-1))
	{
	  startJob();
	  start_counter++;
	}
		
	// Add new point to end of moving queue once first point in current queue has been executed
	FOREVER
	{
	  if (*motion_allowed == false)
	  {
	    delete this;
		return RC_MOTION_INTERRUPT;
	  }
	  mpGetCurJob(&task_data, &cur_job_data);
	  cur_iter = cur_job_data.usJobLine - 2; // "-2" accounts for NOP and LABEL lines in job
	  if (cur_iter == next_iter)
	  {
	    if (cur_iter == 0)
		{
		  next_point.usIndex = QSIZE-1;
		  next_iter++;
		}
		else if (cur_iter >= 1 && cur_iter <= QSIZE-2)
		{
		  next_point.usIndex = cur_iter - 1;
		  next_iter++;
		}
		else if (cur_iter == QSIZE-1)
		{
		  next_point.usIndex = cur_iter - 1;
		  next_iter = 0;
		}
		for (SHORT i = 0; i < 8; i++)
		{
		  next_point.ulValue[i+2] = pulse_coords[i];
		}
		if (mpPutPosVarData(&next_point, 1) == ERROR) // Write next trajectory point position
		  return RC_MP_ERROR;
		joint_speed.usIndex = next_point.usIndex;
	    joint_speed.ulValue = velocity;
		if (mpPutVarData(&joint_speed, 1) == ERROR) // Write next trajectory point velocity
		  return RC_MP_ERROR;
		break;
	  }
	}
  }
	
  // Now that point has been added, return result
  return RC_SUCCESS;
}

void PVarQ::startJob(void)
// Turns on servo power and starts job
{
  printf("Starting job");
  printf("\n");
  // Check if servo power is on. Keep trying to turn it on until it is.
  FOREVER
  {
    FOREVER
	{
	  if (mpGetServoPower(&servo_power_info) == OK);
        break;
	}
	if (servo_power_info.sServoPower == OFF)
	  mpSetServoPower(&servo_power_on, &servo_power_error);
	else
	  break;
  }
	
  // Turn off Hold status if it's on
  hold_data.sHold = OFF;
  FOREVER
  {
    if (mpHold(&hold_data, &hold_error) == OK)
	  break;
  }
	
  // Keep trying to start job until able to do so
  FOREVER
  {
    if (mpStartJob(&job_data, &job_error) == OK)
	  break;
  }
}