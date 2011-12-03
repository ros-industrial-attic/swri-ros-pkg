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
 
 #include "ros_conversion.h"
 #include "log_wrapper.h"
 // Simple method for switching between robots.  TODO: Come up with a beter
 // method of supporting multiple robots.
 #define SIA_10D

using namespace industrial::joint_position;

namespace motoman
{

namespace ros_conversion
{

 
 #ifdef SIA_10D
 
// TODO: Not sure why some of these numbers are negative.  This is how they
// came from motoman.  The direction will be verified relative to the physical
// joint markings on the robot.

/*  CONVERSION FACTORS SENT FROM MOTOMAN
------------------------------------------------------------------------------
S	Resolution (pulse/deg)	1024	    pulses/deg	58670.87822	    pulses/rad
L	Resolution (pulse/deg)	-1024	    pulses/deg	-58670.87822	pulses/rad
U	Resolution (pulse/deg)	1149.1556	pulses/deg	65841.76588	    pulses/rad
R	Resolution (pulse/deg)	-1149.1556	pulses/deg	-65841.76588	pulses/rad
B	Resolution (pulse/deg)	1149.1556	pulses/deg	65841.76588	    pulses/rad
T	Resolution (pulse/deg)	-580.2667	pulses/deg	-33246.8329	    pulses/rad
7	Resolution (pulse/deg)	1149.1556	pulses/deg	65841.76588	    pulses/rad
------------------------------------------------------------------------------
*/
const float S_PULSE_TO_RAD	= 58670.87822;	    // pulses/rad
const float L_PULSE_TO_RAD	= 58670.87822;	    // pulses/rad
const float U_PULSE_TO_RAD	= 65841.76588;	    // pulses/rad
const float R_PULSE_TO_RAD  = 65841.76588;	    // pulses/rad
const float B_PULSE_TO_RAD	= 65841.76588;     	// pulses/rad
const float T_PULSE_TO_RAD	= 33246.8329;	    // pulses/rad
const float E_PULSE_TO_RAD	= 65841.76588;	    // pulses/rad

#endif //SIA_10D


float toPulses(float radians, MotomanJointIndex joint)
{
    float rtn = 0.0;
    switch (joint)
    {
      case MotomanJointIndexes::S:
         rtn = radians * S_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::L:
         rtn = radians * L_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::U:
         rtn = radians * U_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::R:
         rtn = radians * R_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::B:
         rtn = radians * B_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::T:
         rtn = radians * T_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::E:
         rtn = radians * E_PULSE_TO_RAD;
         break;
         
      default:
        rtn = radians;
    }
        return rtn;
}

float toRadians(float pulses, MotomanJointIndex joint)
{
    float rtn = 0.0;
    switch (joint)
    {
      case MotomanJointIndexes::S:
         rtn = pulses / S_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::L:
         rtn = pulses / L_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::U:
         rtn = pulses / U_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::R:
         rtn = pulses / R_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::B:
         rtn = pulses / B_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::T:
         rtn = pulses / T_PULSE_TO_RAD;
         break;
         
      case MotomanJointIndexes::E:
         rtn = pulses / E_PULSE_TO_RAD;
         break;
         
      default:
        rtn = pulses;
    }
        return rtn;
}


void toRosJointOrder(JointPosition & joints)
{
    LOG_DEBUG("Swapping to ROS joint order");
    JointPosition swap;
    swap.setJoint(RosJointIndexes::S, joints.getJoint(MotomanJointIndexes::S));
    swap.setJoint(RosJointIndexes::L, joints.getJoint(MotomanJointIndexes::L));
    swap.setJoint(RosJointIndexes::U, joints.getJoint(MotomanJointIndexes::U));
    swap.setJoint(RosJointIndexes::R, joints.getJoint(MotomanJointIndexes::R));
    swap.setJoint(RosJointIndexes::B, joints.getJoint(MotomanJointIndexes::B));
    swap.setJoint(RosJointIndexes::T, joints.getJoint(MotomanJointIndexes::T));
    swap.setJoint(RosJointIndexes::E, joints.getJoint(MotomanJointIndexes::E));
    joints.copyFrom(swap);
}

void toMotomanJointOrder(JointPosition & joints)
{
    LOG_DEBUG("Swapping to motoman joint order");
    JointPosition swap;
    swap.setJoint(MotomanJointIndexes::S, joints.getJoint(RosJointIndexes::S));
    swap.setJoint(MotomanJointIndexes::L, joints.getJoint(RosJointIndexes::L));
    swap.setJoint(MotomanJointIndexes::U, joints.getJoint(RosJointIndexes::U));
    swap.setJoint(MotomanJointIndexes::R, joints.getJoint(RosJointIndexes::R));
    swap.setJoint(MotomanJointIndexes::B, joints.getJoint(RosJointIndexes::B));
    swap.setJoint(MotomanJointIndexes::T, joints.getJoint(RosJointIndexes::T));
    swap.setJoint(MotomanJointIndexes::E, joints.getJoint(RosJointIndexes::E));
    joints.copyFrom(swap);
}

void getMotomanFbPos(JointPosition & pos)
{
    LONG getPulseRtn = 0;
    
    MP_CTRL_GRP_SEND_DATA sData;
    MP_FB_PULSE_POS_RSP_DATA rData;
    
    // Get feedback for first robot
    sData.sCtrlGrp = 0;
        
    // Get pulse data from robot
    getPulseRtn = mpGetFBPulsePos (&sData,&rData);
    
    if (0 == getPulseRtn)
    {
        toJointPosition(rData, pos);
    }
    else
    {
        LOG_ERROR("Failed to get pulse feedback position: %u", getPulseRtn);
    }
}


void getRosFbPos(JointPosition & pos)
{
    getMotomanFbPos(pos);
    toRosJointOrder(pos);
}

void toJointPosition(MP_FB_PULSE_POS_RSP_DATA & src, JointPosition & dest)
{    

    int minJointSize = 0;
    int jointPosSize = dest.getMaxNumJoints();
    
    // Check for size constraints on Postion data
    if (MAX_PULSE_AXES >= jointPosSize)
    {
        LOG_WARN("Number of pulse axes: %u exceeds joint position size: %u",
            MAX_PULSE_AXES, dest.getMaxNumJoints());
    }
    
    // Determine which buffer is the smallest and only copy that many members
    if (jointPosSize > MAX_PULSE_AXES)
    {
        minJointSize = MAX_PULSE_AXES;
    }
    else
    {
        minJointSize = jointPosSize;
    }
    
    for(int i = 0; i < minJointSize; i++)
    {
          dest.setJoint(i, toRadians(src.lPos[i], (MotomanJointIndex)i));
    }
    
}

void toMpPosVarData(USHORT posVarIndex, JointPosition & src, MP_POSVAR_DATA & dest)
{
  const int MOTOMAN_AXIS_SIZE = 8;
  LONG pulse_coords[MOTOMAN_AXIS_SIZE];
  
  motoman::ros_conversion::toMotomanJointOrder(src);
  for (int i = 0; i < MOTOMAN_AXIS_SIZE; i++)
  {
    pulse_coords[i] = motoman::ros_conversion::toPulses(src.getJoint(i), 
        (motoman::ros_conversion::MotomanJointIndex)i);
  }
  
  dest.usType = MP_RESTYPE_VAR_ROBOT;  // All joint positions are robot positions
  dest.usIndex = posVarIndex;          // Index within motoman position variable data table
  dest.ulValue[0] = 0;                 // Position is in pulse counts
  
  for (SHORT i = 0; i < MOTOMAN_AXIS_SIZE; i++)
  {
	dest.ulValue[i+2] = pulse_coords[i];  // First two values in ulValue reserved
  }
}

} //ros_conversion
} //motoman