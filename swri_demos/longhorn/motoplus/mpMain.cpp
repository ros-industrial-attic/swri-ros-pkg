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
#include "ros_conversion.h"
#include "mp_default_main.h"

#include "tcp_server.h"
#include "gripper_handler.h"
#include "trajectory_download_handler.h"
#include "longhorn.h"
#include "message_manager.h"
#include "controller.h"


// Global controller
motoman::controller::Controller rbtCtrl;


void gripperServer(void)
{


    using namespace industrial::tcp_server;
    using namespace industrial::simple_socket;
    using namespace industrial::message_manager;
    using namespace industrial::simple_message;
    using namespace industrial::longhorn;
    using namespace longhorn::gripper_handler;
    
    TcpServer connection;
    GripperHandler gHandler;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::IO);
    connection.makeConnect();
    
    manager.init(&connection);
    
    gHandler.init(&connection, &rbtCtrl);
    manager.add(&gHandler);
    manager.spin();
 

    
}

void motionDownloadServer(void)
{

    
    using namespace industrial::tcp_server;
    using namespace industrial::simple_socket;
    using namespace industrial::message_manager;
    using namespace industrial::simple_message;
    using namespace industrial::longhorn;
    using namespace longhorn::trajectory_download_handler;
    
    TcpServer connection;
    
    TrajectoryDownloadHandler tdHandler;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::MOTION);
    connection.makeConnect();
    
    manager.init(&connection);
    
    tdHandler.init(&connection, &rbtCtrl);
    
    manager.add(&tdHandler);
    manager.spin();
    

    
}




using namespace motoman::mp_default_main;
using namespace motoman::ros_conversion;

int motion_server_task_ID; \
int system_server_task_ID; \
int state_server_task_ID; \
int gripper_server_task_ID; \
extern "C" void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{  
  initJointConversion( MotomanRobotModels::SIA_20D );
  motion_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)motionServer, 
                        arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
  //motion_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)motionDownloadServer, 
  //                      arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
  //system_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)systemServer, 
  //					arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10); 
  state_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
                                    (FUNCPTR)stateServer, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10); 
  gripper_server_task_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)gripperServer, 
  					arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10); 
  mpExitUsrRoot; //Ends the initialization task. 
} 
