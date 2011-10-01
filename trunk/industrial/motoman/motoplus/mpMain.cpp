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

//mpMain.cpp
//
//This contains mpUsrRoot which is the entry point for your MotoPlus application

#include "motoPlus.h"
#include "definitions.h"
#include "p_var_q.h"
#include "ros_socket.h"
#include "utils.h"
#include "system.h"

// Using directives
using utils::arrayIntToChar;
using utils::arrayCharToInt;

// Global data
int motion_server_task_ID;
int system_server_task_ID;
bool motion_allowed_var = true;
bool* motion_allowed = &motion_allowed_var;

// Function prototypes
void motionServer();
void parseMotionMessage(LONG recv_message[], ROSSocket *sock);
void systemServer();
void parseSystemMessage(LONG recv_message[], ROSSocket *sock);

// Function definitions
extern "C" void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{	
  motion_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)motionServer,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
  system_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)systemServer,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
  mpExitUsrRoot; //Ends the initialization task.
}

void motionServer(void)
// Persistent UDP server that receives motion messages from Motoros node (ROS interface) and relays to parseMotionMessage
{
  LONG bytes_recv;
  LONG recv_message[11];
	
  FOREVER
  {
    ROSSocket *sock = new ROSSocket(MOTION_PORT); // Socket object
	printf("Motion socket created");
	printf("\n");
		
	FOREVER
	{
	  memset(recv_message, 0, sizeof(recv_message));
	  bytes_recv = sock->recvMessage(recv_message);
	  if (bytes_recv == ERROR)
	  {
        printf("Motion socket receive error");
		printf("\n");
		break;
	  }
	  parseMotionMessage(recv_message, sock);
	}
	delete sock;
	printf("Motion socket deleted");
	printf("\n");
  }
}

void parseMotionMessage(LONG recv_message[], ROSSocket *sock)
// Receives messages from motion UDP server and decides next action based on command ID of message
{
  LONG command_ID = recv_message[0];
  static PVarQ *pvq = NULL;
  LONG reply_code;
	
  switch(command_ID) // Decide response based on command ID of message
  {
    case CMD_INIT_PVQ:
	  if (pvq == NULL)
	  {
	    *motion_allowed = true;
		pvq = new PVarQ(sock, motion_allowed); // Initialize new position variable queue motion
		sock->sendMessage(CMD_INIT_PVQ, RC_SUCCESS, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
		printf("New PVQ");
		printf("\n");
	  }
	  else
		sock->sendMessage(CMD_INIT_PVQ, RC_AE, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
	  break;
    case CMD_ADD_POINT_PVQ:
	  if (pvq != NULL)
	  {
		reply_code = pvq->addPointPVQ(recv_message); // Add new point to position variable queue
		sock->sendMessage(CMD_ADD_POINT_PVQ, reply_code, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
	  }
	  else
		sock->sendMessage(CMD_ADD_POINT_PVQ, RC_NOT_INIT, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
	  break;
    case CMD_END_PVQ:
	  if (pvq != NULL)
	  {
		delete pvq; // End position variable queue motion
		printf("%d ",*pvq);
		pvq = NULL;
		sock->sendMessage(CMD_END_PVQ, RC_SUCCESS, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
		printf("PVQ Deleted");
		printf("\n");
		printf("%d",pvq);
		printf("\n");
	  }
	  break;
  }
}

void systemServer(void)
// Persistent UDP server that receives system messages from Motoros node (ROS interface) and relays to parseSystemMessage
{
  LONG bytes_recv;
  LONG recv_message[11];
	
  FOREVER
  {
    ROSSocket *sock = new ROSSocket(SYSTEM_PORT); // Socket object
	printf("System socket created");
	printf("\n");
		
	FOREVER
	{
	  memset(recv_message, 0, sizeof(recv_message));
	  bytes_recv = sock->recvMessage(recv_message);
	  if (bytes_recv == ERROR)
	  {
        printf("System socket receive error");
        printf("\n");
		break;
	  }
	  parseSystemMessage(recv_message, sock);
	}
	delete sock;
	printf("System socket deleted");
	printf("\n");
  }
}

void parseSystemMessage(LONG recv_message[], ROSSocket *sock)
// Receives messages from system UDP server and decides next action based on command ID of message
{
  LONG command_ID = recv_message[0];
  static System *sys = new System(sock, motion_allowed);
  LONG reply_code;
  LONG reply_message[11];
	
  switch(command_ID) // Decide response based on command ID of message
  {
    case CMD_HOLD:
	  reply_code = sys->hold(true);
	  sock->sendMessage(CMD_HOLD, reply_code, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED);
	  break;
	case CMD_GET_FB_PULSE:
	  sys->getFBPulse(reply_message);
	  sock->sendMessage(reply_message);
	  break;
	case CMD_GET_FB_SPEED:
	  sys->getFBSpeed(reply_message);
	  sock->sendMessage(reply_message);
	  break;
	case CMD_GET_TORQUE:
	  sys->getTorque(reply_message);
	  sock->sendMessage(reply_message);
	  break;
  }
}