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

#ifndef ROS_CONVERSION_H
#define ROS_CONVERSION_H

#include "joint_position.h"
#include "motoPlus.h"

namespace motoman
{
namespace ros_conversion
{

/**
 * \brief Enumeration of motoman joint indicies (as returned by the getPos
 * library calls.
 */
namespace MotomanJointIndexes
{
enum MotomanJointIndex
{
  S = 0, 
  L = 1, 
  U = 2,
  R = 3,
  B = 4,
  T = 5,
  E = 6,
  COUNT = E
};
}
typedef MotomanJointIndexes::MotomanJointIndex MotomanJointIndex;

/**
 * \brief Enumeration of ROS joint indicies.  In reality these are not
 * fixed.  The ROS message structure maps joints by name and theoretically
 * would allow for any order.  Generally the order from base to tip is 
 * maintained (This is what the enumeration assumes).
 */
namespace RosJointIndexes
{
enum RosJointIndex
{
  S = 0, 
  L = 1, 
  E = 2,  
  U = 3,
  R = 4,
  B = 5,
  T = 6,
  COUNT = T
};
}
typedef RosJointIndexes::RosJointIndex RosJointIndex;

//TODO: Standardize function calls such that JointPositions are always in ROS
//      order and MP types are always in motoman order.  The order of the
//      JointPosition type can change within a function, but when a variable
//      of that type enters or leaves a function is should always be in ROS
//      order.  Move the appropriate function prototypes below to the src
//      file in order to hide the ordering details (essentially make them private)
float toPulses(float radians, MotomanJointIndex joint);
float toRadians(float pulses, MotomanJointIndex joint);

void toRosJointOrder(industrial::joint_position::JointPosition & joints);
void toMotomanJointOrder(industrial::joint_position::JointPosition & joints);

void getMotomanFbPos(industrial::joint_position::JointPosition & pos);
void getRosFbPos(industrial::joint_position::JointPosition & pos);

void toJointPosition(MP_FB_PULSE_POS_RSP_DATA & src, 
    industrial::joint_position::JointPosition & dest);
    
void toMpPosVarData(USHORT posVarIndex, industrial::joint_position::JointPosition & src, 
    MP_POSVAR_DATA & dest);
    
    
} //ros_conversion
} //motoman


#endif //ROS_CONVERSION_H