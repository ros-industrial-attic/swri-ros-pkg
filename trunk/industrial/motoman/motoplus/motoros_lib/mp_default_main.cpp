/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#include "mp_default_main.h"
#include "log_wrapper.h"
#include "motoPlus.h"

namespace motoman
{
namespace mp_default_main
{

#include "motoPlus.h"
#include "p_var_q.h"

#include "log_wrapper.h"
#include "tcp_server.h"
#include "message_manager.h"
#include "input_handler.h"
#include "joint_motion_handler.h"

#include "udp_server.h"
#include "ros_conversion.h"
#include "joint_data.h"
#include "joint_message.h"
#include "simple_message.h"


// Pulse to radian conversion factors (initialized on startup)
float S_PULSE_TO_RAD	= 0;	    // pulses/rad
float L_PULSE_TO_RAD	= 0;	    // pulses/rad
float U_PULSE_TO_RAD	= 0;	    // pulses/rad
float R_PULSE_TO_RAD    = 0;	    // pulses/rad
float B_PULSE_TO_RAD	= 0;     	// pulses/rad
float T_PULSE_TO_RAD	= 0;	    // pulses/rad
float E_PULSE_TO_RAD	= 0;	    // pulses/rad




void initJointConversion(MotomanRobotModel model_number)
{
    
    LOG_INFO("Initializing joint conversion factors for: ");
    switch (model_number)
    {
    case MotomanRobotModels::SIA_10D:
        LOG_INFO("SIA_10D: %d", model_number);
        S_PULSE_TO_RAD	= 58670.87822;	    
        L_PULSE_TO_RAD	= 58670.87822;	 
        U_PULSE_TO_RAD	= 65841.76588;	    
        R_PULSE_TO_RAD  = 65841.76588;	  
        B_PULSE_TO_RAD	= 65841.76588;    
        T_PULSE_TO_RAD	= 33246.8329;	  
        E_PULSE_TO_RAD	= 65841.76588;	
        break;
    
    default:
        LOG_ERROR("Failed to initialize conversion factors for model: %d", model_number);
        break;
    }
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
    using namespace industrial::joint_data;
    using namespace industrial::simple_message;
    using namespace motoman::ros_conversion;
    
    // Using TPC server for debugging (this should really be UDP)
    TcpServer connection;
    JointData rosJoints;
    JointMessage msg;
    SimpleMessage simpMsg;
    
    const int period = 100; //ticks
    
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
/*

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
 */

    
}


} //mp_wrapper
} //motoman
