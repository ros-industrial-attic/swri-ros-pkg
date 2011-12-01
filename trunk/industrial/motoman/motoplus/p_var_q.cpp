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

#include "p_var_q.h"
#include "joint_position.h"
#include "joint_motion_handler.h"
#include "ros_conversion.h"
#include "log_wrapper.h"

using motoman::joint_motion_handler;
using motoman::ros_conversion;
using industrial::joint_position;

PVarQ::PVarQ()
{	

  // Set up point variable
  pointData_.usType = MP_RESTYPE_VAR_ROBOT;
  
  // Set integer variables
  jointSpeedData_.usType = MP_RESTYPE_VAR_I;
  
  motionIndexInfo_.usType = MP_RESTYPE_VAR_I;
  motionIndexInfo_.usIndex = MOTION_POINTER;
  
  bufferIndexInfo_.usType = MP_RESTYPE_VAR_I;
  bufferIndexInfo_.usIndex = BUFFER_POINTER;
  bufferIndexData_.usType = MP_RESTYPE_VAR_I;
  bufferIndexData_.usIndex = BUFFER_POINTER;
  
  // TODO: Should check constants such as Q_SIZE, MOTION_POINTER & BUFFER_POINTER for
  // consitency
	
}

PVarQ::~PVarQ(void)
{
}


void PVarQ::init(industrial::joint_position::JointPosition & point, double velocity_percent)
{
  // Seed the intial point - this is required because upon startup, the indexes are zero and
  // therefore the intial point never gets set.
  setPosition(0, point, velocity_percent);
}

void PVarQ::addPoint(industrial::joint_position::JointPosition & joints)
{

  // Wait until buffer is not full
  while(this->bufferFull()) {
      LOG_DEBUG("Waiting for buffer to not be full, retrying...");
      mpTaskDelay(this->BUFFER_POLL_TICK_DELAY);
  };
  
  setNextPosition(joints, VELOCITY);
  incBufferIndex();
	
}

int PVarQ::bufferSize()
{
  int motionIdx = this->getMotionIndex();
  int bufferIdx = this->getBufferIndex();
  int rtn = 0;
  
  if (motionIdx > bufferIdx)
  {
    LOG_ERROR("Motion index: %d is greater than Buffer index: %d, returning empty buffer size", motionIdx, bufferIdx);
    rtn = 0;
  }
  else
  {
    rtn = bufferIdx - motionIdx;
  }
  
  LOG_DEBUG("Buffer size: %d, buffer idx: %d, motion idx: %d", rtn, bufferIdx, motionIdx);
  return rtn;  
}
    
    
int PVarQ::getMotionIndex()
{
  LONG val = 0;
  
  while (mpGetVarData ( &(this->motionIndexInfo_), &val, 1 ) == ERROR) {
    LOG_ERROR("Failed to retreive motion index, retrying...");
    mpTaskDelay(this->VAR_POLL_TICK_DELAY);
  };
  return val;
}
    
int PVarQ::getBufferIndex()
{
  LONG val = 0;
  
  while (mpGetVarData ( &(this->bufferIndexInfo_), &val, 1 ) == ERROR) {
    LOG_ERROR("Failed to retreive buffer index, retrying...");
    mpTaskDelay(this->VAR_POLL_TICK_DELAY);
  };
  return val;
}

  
int PVarQ::getMotionPosIndex()
{
  return (this->getMotionIndex() % this->posVarQueueSize());
}
  
int PVarQ::getBufferPosIndex()
{
  return (this->getBufferIndex() % this->posVarQueueSize());
}

int PVarQ::getNextBufferPosIndex()
{
  return ((this->getBufferIndex() + 1) % this->posVarQueueSize());
}
    
bool PVarQ::bufferFull()
{
  bool rtn = false;
  int bufferSize = this->bufferSize();
  int maxBufferSize = this->maxBufferSize();
  if (bufferSize >= maxBufferSize)
  {
    LOG_DEBUG("Buffer is full, size: %d, max size: %d",
        bufferSize, maxBufferSize);
    rtn = true;
  }
  return rtn;
}

bool PVarQ::bufferEmpty()
{
  bool rtn = false;
  if (this->bufferSize() <= 0)
  {
    LOG_DEBUG("Buffer is empty");
    rtn = true;
  }
  return rtn;
}

void PVarQ::incBufferIndex()
{
  int bufferIdx = this->getBufferIndex();
  this->bufferIndexData_.ulValue = bufferIdx + 1;
  
  LOG_DEBUG("Incrementing buffer index from %d to %d", bufferIdx, this->bufferIndexData_.ulValue);
  
  while (mpPutVarData ( &(this->bufferIndexData_), 1 ) == ERROR) {
    LOG_ERROR("Failed to increment buffer index, retrying...");
    mpTaskDelay(this->VAR_POLL_TICK_DELAY);
  };
}


void PVarQ::setNextPosition(industrial::joint_position::JointPosition & point, double velocity_percent)
{
  setPosition(this->getNextBufferPosIndex(), point, velocity_percent); 
}


void PVarQ::setPosition(int index, industrial::joint_position::JointPosition & point, 
    double velocity_percent)
{
  const double VELOCITY_CONVERSION = 100.0;
  int convertedVelocity = 0;
  
  LOG_DEBUG("Setting joint position, index: %d", index);
  motoman::ros_conversion::toMpPosVarData(index, point, this->pointData_);
  
  while (mpPutPosVarData ( &(this->pointData_), 1 ) == ERROR) {
    LOG_ERROR("Failed set position variable, index: %d, retrying...", index);
    mpTaskDelay(this->VAR_POLL_TICK_DELAY);
  };
  
  convertedVelocity = (int)(velocity_percent * VELOCITY_CONVERSION);
  LOG_DEBUG("Converting percent velocity: %g to motoman integer value: %d", 
    velocity_percent, convertedVelocity);
  this->jointSpeedData_.ulValue = convertedVelocity;
  this->jointSpeedData_.usIndex = index;
  
  LOG_DEBUG("Setting velocity, index: %d, value: %d", this->jointSpeedData_.usIndex, 
    this->jointSpeedData_.ulValue);
    
  while (mpPutVarData ( &(this->jointSpeedData_), 1 ) == ERROR) {
    LOG_ERROR("Failed to set position varaible, index: %d, retrying...", this->jointSpeedData_.usIndex);
    mpTaskDelay(this->VAR_POLL_TICK_DELAY);
  };
}