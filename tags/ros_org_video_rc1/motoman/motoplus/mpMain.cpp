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
#include "p_var_q.h"
#include "system.h"

#include "log_wrapper.h"
#include "tcp_server.h"
#include "message_manager.h"
#include "input_handler.h"
#include "joint_motion_handler.h"

#include "udp_server.h"
#include "ros_conversion.h"
#include "joint_position.h"
#include "joint_message.h"
#include "simple_message.h"

// Global data
int motion_server_task_ID;
int system_server_task_ID;
int state_server_task_ID;
int io_server_task_ID;

// Function prototypes
void motionServer();
void systemServer();
void stateServer();
void ioServer();

// Function definitions
extern "C" void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{	
    
  motion_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)motionServer,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
						
  //system_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)systemServer,
	//					arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
  
  state_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)stateServer,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
  
  //io_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)ioServer,
	//					arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
						
  mpExitUsrRoot; //Ends the initialization task.
}

void motionServer(void)
// Persistent UDP server that receives motion messages from Motoros node (ROS interface) and relays to parseMotionMessage
{
    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::message_manager;
    using namespace industrial::simple_message;
    using namespace motoman::joint_motion_handler;
    
    TcpServer connection;
    JointMotionHandler jmHandler;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::MOTION);
    connection.makeConnect();
    
    manager.init(&connection);
    
    jmHandler.init(StandardMsgTypes::JOINT, &connection);
    manager.add(&jmHandler);
    manager.spin();
	

}

void systemServer(void)
// Persistent UDP server that receives system messages from Motoros node (ROS interface) and relays to parseSystemMessage
{

    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::message_manager;
    
    TcpServer connection;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::SYSTEM);
    connection.makeConnect();
    
    manager.init(&connection);
    manager.spin();

}


void stateServer(void)
{

    using namespace industrial::simple_socket;
    using namespace industrial::udp_server;
    using namespace industrial::tcp_server;
    using namespace industrial::joint_message;
    using namespace industrial::joint_position;
    using namespace industrial::simple_message;
    using namespace motoman::ros_conversion;
    
    // Using TPC server for debugging (this should really be UDP)
    TcpServer connection;
    JointPosition rosJoints;
    JointMessage msg;
    SimpleMessage simpMsg;
    
    const int period = 500; //ticks
    
    connection.init(StandardSocketPorts::STATE);
    
    FOREVER
    {
      connection.makeConnect();
      
      while(connection.isConnected())
      {
        getRosFbPos(rosJoints);
        msg.init(0, rosJoints);
        msg.toTopic(simpMsg);
        connection.sendMsg(simpMsg);
        
        mpTaskDelay(period);
        
        
      }
      

    }
}


void ioServer(void)
{

    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::message_manager;
    using namespace industrial::simple_message;
    using namespace motoman::input_handler;
    
    TcpServer connection;
    InputHandler iHandler;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::IO);
    connection.makeConnect();
    
    manager.init(&connection);
    
    iHandler.init(StandardMsgTypes::WRITE_OUTPUT, &connection);
    manager.add(&iHandler);
    manager.spin();

    
}