/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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
 *       * Neither the name of the Southwest Research Institute, nor the names
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


#ifndef TRAJECTORY_DOWNLOAD_HANDLER_H
#define TRAJECTORY_DOWNLOAD_HANDLER_H

#include "message_handler.h"
#include "joint_traj.h"
#include "joint_traj_pt_message.h"
#include "controller.h"

namespace armadillo
{
namespace trajectory_download_handler
{


/**
* \brief job name
*/
//TODO: Should be "class static const" not macro
#define JOB_NAME "ROS_I_TRAJ.JBI"  

/**
* \brief size of job buffer
*/
//TODO: Should be "class static const" not macro
#define JOB_BUFFER_SIZE_ 5000   
  
  
/**
 * \brief Message handler that handles the recieiving of entire trajectories
 * and trajectory inform job execution.
 */
//* TrajectoryDownloadHandler
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class TrajectoryDownloadHandler : public industrial::message_handler::MessageHandler
{

public:
  /**
* \brief Class initializer
*
* \param connection simple message connection that will be used to send replies.
* \param controller object used for enabling/disabling motion
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
            motoman::controller::Controller* ctrl);

protected:


  /**
* \brief Class initializer (Direct call to base class with the same name)
* I couldn't get the "using" form to work/
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(int msg_type, industrial::smpl_msg_connection::SmplMsgConnection* connection)
{ return MessageHandler::init(msg_type, connection);};

 /**
  * \brief Callback executed upon receiving message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage & in);
 
  /**
  * \brief Handle joint trajectory start message
  *
  * \param in incoming joint message message (assumes sequence is set to 
  * START_TRAJECTORY_DOWNLOAD)
  *
  */
 void startTrajectory(industrial::joint_traj_pt_message::JointTrajPtMessage & jMsg);
 
 /**
  * \brief Handle joint trajectory start message
  *
  * \param in incoming joint message message (assumes sequence is set to 
  * START_TRAJECTORY_DOWNLOAD)
  *
  */
 void endTrajectory(industrial::joint_traj_pt_message::JointTrajPtMessage & jMsg);
  

   /**
* \brief Controller object for handing jobs and motion
*/
 motoman::controller::Controller* ctrl_;
 
/**
   * \brief joint trajectory (internal buffer)
   */
  industrial::joint_traj::JointTraj traj_;
  
 /**
   * \brief job sting buffer (limited to JOB_BUFFER_SIZE_ characters);
   */
   char jobBuffer_[JOB_BUFFER_SIZE_];
 
};
 


}//trajectory_download_handler
}//motoman


#endif // TRAJECTORY_DOWNLOAD_HANDLER_H