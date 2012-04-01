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


#ifndef GRIPPER_HANDLER_H
#define GRIPPER_HANDLER_H

#include "message_handler.h"
#include "controller.h"
#include "motoPlus.h"

namespace longhorn
{
namespace gripper_handler
{

/**
 * \brief Message handler that responds to gripper operation requests
 */
//* GripperHandler
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class GripperHandler : public industrial::message_handler::MessageHandler
{

public:
  /**
* \brief Class initializer
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
            motoman::controller::Controller* ctrl);

protected:

 // Gripper constants
 //
 // The gripper operation is achieved using operation bits and
 // status bit for feedback.
 //
 // For and OPEN/CLOSE move here is the expected order of operations
 // 1. Set OPEN/CLOSE operations bit
 // 2. Wait for OPENING/CLOSING status bit
 // 3. Wait for MOTION_IN_PROGRESS status bit
 // 4. Wait for MOTION_COMPLETE_BIT status bit
 
 // Operations
 static const int GRIPPER_OUT_BASE_ = 40;
 static const int GRIPPER_INIT_ = GRIPPER_OUT_BASE_ + 0;
 static const int GRIPPER_PINCH_MODE_ = GRIPPER_OUT_BASE_ + 1;
 static const int GRIPPER_OPEN_ = GRIPPER_OUT_BASE_ + 3;
 static const int GRIPPER_CLOSE_ = GRIPPER_OUT_BASE_ + 4;
 
 // Status
 static const int GRIPPER_IN_BASE_ = 40;
 static const int GRIPPER_INITIALIZED_ = GRIPPER_IN_BASE_ + 0;
 static const int GRIPPER_IN_PINCH_MODE_ = GRIPPER_IN_BASE_ + 1;
 static const int GRIPPER_OPENING_ = GRIPPER_IN_BASE_ + 3;
 static const int GRIPPER_CLOSING_ = GRIPPER_IN_BASE_ + 4;
 static const int GRIPPER_MOTION_IN_PROGRESS_ = GRIPPER_IN_BASE_ + 5;
 static const int GRIPPER_MOTION_COMPLETE_ = GRIPPER_IN_BASE_ + 6;

 // Delay between outputs are set and motion starts (this allows status
 // inputs to update as well.  Blocking on other status indicators did
 // not work.
 static const int MOTION_START_DELAY = 50;
 
 
   /**
* \brief Controller object for setting/reading IO
*/
 motoman::controller::Controller* ctrl_;

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
 
};

}//gripper_handler
}//motoman


#endif /* INPUT_HANDLER_H_ */