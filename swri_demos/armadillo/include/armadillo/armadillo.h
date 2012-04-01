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

#ifndef ARMADILLO_H_
#define ARMADILLO_H_

#ifdef ROS
#include "simple_message/simple_message.h"
#endif

#ifdef MOTOPLUS
#include "simple_message.h"
#endif

namespace industrial
{
namespace armadillo
{


/**
 * \brief Enumeration of standard message types (supported by all platforms).
 * In addition, each robot interface will support it's own message types.
 */
namespace ArmadilloMsgTypes
{
  enum ArmadilloMsgType
  {
    GRIPPER = industrial::simple_message::StandardMsgTypes::SWRI_MSG_BEGIN
  };
}
typedef ArmadilloMsgTypes::ArmadilloMsgType ArmadilloMsgType;

} //armadillo
} //industrial

#endif /* ARMADILLO_H_ */
