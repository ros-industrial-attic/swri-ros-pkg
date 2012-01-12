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

#include "gripper_handler.h"
#include "shared_types.h"
#include "log_wrapper.h"
#include "armadillo.h"
#include "gripper_message.h"
#include "motoPlus.h"

using namespace industrial::simple_message;
using namespace industrial::shared_types;
using namespace industrial::armadillo;
using namespace industrial::gripper_message;

namespace armadillo
{
namespace gripper_handler
{


bool GripperHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  return this->init(ArmadilloMsgTypes::GRIPPER, connection);
}

bool GripperHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{

  bool rtn = false;
  bool unloadStat = false;
  shared_int temp;
  int i = 0;
  ReplyType code = ReplyTypes::INVALID;
  SimpleMessage reply;
  GripperMessage gMsg;
  
  gMsg.init(in);
  
  switch( gMsg.operation_ )
  {
  case GripperOperationTypes::INIT:
    LOG_INFO("Initializing gripper");
    setDigitalOut(this->GRIPPER_INIT_, true);
    mpTaskDelay(MOTION_START_DELAY);
    waitDigitalIn(this->GRIPPER_INITIALIZED_, true);
    code = ReplyTypes::SUCCESS;
    break;
    
  case GripperOperationTypes::OPEN:
    LOG_DEBUG("Opening gripper");
    setDigitalOut(this->GRIPPER_OPEN_, true);
    setDigitalOut(this->GRIPPER_CLOSE_, false);
    mpTaskDelay(MOTION_START_DELAY);
    waitDigitalIn(this->GRIPPER_OPENING_, true);
    waitDigitalIn(this->GRIPPER_MOTION_IN_PROGRESS_, true);
    waitDigitalIn(this->GRIPPER_MOTION_COMPLETE_, true);
    code = ReplyTypes::SUCCESS;
    break;
    
  case GripperOperationTypes::CLOSE:
    LOG_DEBUG("Closing gripper");
    setDigitalOut(this->GRIPPER_OPEN_, false);
    setDigitalOut(this->GRIPPER_CLOSE_, true);
    mpTaskDelay(MOTION_START_DELAY);
    waitDigitalIn(this->GRIPPER_CLOSING_, true);
    waitDigitalIn(this->GRIPPER_MOTION_IN_PROGRESS_, true);
    waitDigitalIn(this->GRIPPER_MOTION_COMPLETE_, true);
    code = ReplyTypes::SUCCESS;
    break;
    
  default:
    LOG_ERROR("Unknown gripper request: %d", gMsg.operation_);
    code = ReplyTypes::FAILURE;
    break;
  }
  
  reply.init(ArmadilloMsgTypes::GRIPPER, CommTypes::SERVICE_REPLY, code);
  rtn = this->getConnection()->sendMsg(reply);
  
  return rtn;

}

void GripperHandler::setDigitalOut(int bit_offset, bool value)
{
  LOG_DEBUG("Setting digital out, Bit offset: %d, value: %d", bit_offset, value);
  if ( (bit_offset < this->UNIV_IO_DATA_SIZE_) && 
       ( bit_offset > 0) )
  {  
    MP_IO_DATA data;
    data.ulAddr = this->UNIV_OUT_DATA_START_ + bit_offset;
    data.ulValue = value;
    //TODO: The return result of mpWriteIO is not checked
    mpWriteIO(&data, 1);
  }
  else
  {
    LOG_ERROR("Bit offset: %d, is greater than size: %d", bit_offset, this->UNIV_IO_DATA_SIZE_);
  }
}

 /**
  * \brief Utility function for waiting for a digital input
  * in the Universal input data tabel (most IO is accessible there).
  *
  * \param bit offset in data table (0-2047)
  * \param in incoming message
  *
  */
 void GripperHandler::waitDigitalIn(int bit_offset, bool wait_value)
 {
   LOG_DEBUG("Waiting for digital in, Bit offset: %d, Wait value: %d", bit_offset, wait_value);
   if ( (bit_offset < this->UNIV_IO_DATA_SIZE_) && 
       ( bit_offset > 0) )
  { 
    MP_IO_INFO info;
    info.ulAddr = this->UNIV_IN_DATA_START_ + bit_offset;
    
    USHORT readValue;
    do
    {
      readValue = !wait_value;  
      //TODO: The return result of mpReadIO is not checked
      mpReadIO (&info, &readValue, 1);
      mpTaskDelay(VAR_POLL_TICK_DELAY);
    } while ( ((bool)readValue) != wait_value);
  }
  else
  {
    LOG_ERROR("Bit offset: %d, is greater than size: %d", bit_offset, this->UNIV_IO_DATA_SIZE_);
  }
 }



}//namespace ping_handler
}//namespace motoman


