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
using namespace motoman::controller;

namespace armadillo
{
namespace gripper_handler
{


bool GripperHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
        motoman::controller::Controller* ctrl)
{
  this->ctrl_ = ctrl;
  return this->init(ArmadilloMsgTypes::GRIPPER, connection);
}

bool GripperHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{

  bool rtn = false;
  ReplyType code = ReplyTypes::INVALID;
  SimpleMessage reply;
  GripperMessage gMsg;
  
  gMsg.init(in);
  
  switch( gMsg.operation_ )
  {
  case GripperOperationTypes::INIT:
    LOG_INFO("Initializing gripper");
    this->ctrl_->setDigitalOut(this->GRIPPER_INIT_, true);
    this->ctrl_->delayTicks(MOTION_START_DELAY);
    this->ctrl_->waitDigitalIn(this->GRIPPER_INITIALIZED_, true);
    code = ReplyTypes::SUCCESS;
    break;
    
  case GripperOperationTypes::OPEN:
    LOG_DEBUG("Opening gripper");
    this->ctrl_->setDigitalOut(this->GRIPPER_OPEN_, true);
    this->ctrl_->setDigitalOut(this->GRIPPER_CLOSE_, false);
    this->ctrl_->delayTicks(MOTION_START_DELAY);
    this->ctrl_->waitDigitalIn(this->GRIPPER_OPENING_, true);
    this->ctrl_->waitDigitalIn(this->GRIPPER_MOTION_IN_PROGRESS_, true);
    this->ctrl_->waitDigitalIn(this->GRIPPER_MOTION_COMPLETE_, true);
    code = ReplyTypes::SUCCESS;
    break;
    
  case GripperOperationTypes::CLOSE:
    LOG_DEBUG("Closing gripper");
    this->ctrl_->setDigitalOut(this->GRIPPER_OPEN_, false);
    this->ctrl_->setDigitalOut(this->GRIPPER_CLOSE_, true);
    this->ctrl_->delayTicks(MOTION_START_DELAY);
    this->ctrl_->waitDigitalIn(this->GRIPPER_CLOSING_, true);
    this->ctrl_->waitDigitalIn(this->GRIPPER_MOTION_IN_PROGRESS_, true);
    this->ctrl_->waitDigitalIn(this->GRIPPER_MOTION_COMPLETE_, true);
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



}//namespace gripper_handler
}//namespace motoman


