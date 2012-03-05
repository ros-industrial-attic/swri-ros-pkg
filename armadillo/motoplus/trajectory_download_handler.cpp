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

#include "trajectory_download_handler.h"
#include "shared_types.h"
#include "log_wrapper.h"
#include "armadillo.h"
#include "joint_traj_pt.h"
#include "trajectory_job.h"
#include "motoPlus.h"
#include "smpl_msg_connection.h"

using namespace industrial::simple_message;
using namespace industrial::shared_types;
using namespace industrial::armadillo;
using namespace industrial::joint_data;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::smpl_msg_connection;
using namespace motoman::trajectory_job;
using namespace motoman::controller;

namespace armadillo
{
namespace trajectory_download_handler
{


bool TrajectoryDownloadHandler::init(SmplMsgConnection* connection, Controller* ctrl)
{
  this->ctrl_ = ctrl;
  return this->init(StandardMsgTypes::JOINT_TRAJ_PT, connection);
}

bool TrajectoryDownloadHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{

  bool rtn = false;
  SimpleMessage reply;
  JointTrajPtMessage jMsg;
  
  jMsg.init(in);
  
  switch( jMsg.point_.getSequence() )
  {
  case SpecialSeqValues::START_TRAJECTORY_DOWNLOAD:
    startTrajectory(jMsg);
    rtn = true;
    break;
    
  case SpecialSeqValues::END_TRAJECTORY:
    endTrajectory(jMsg);
    rtn = true;
    break;
    
  case SpecialSeqValues::STOP_TRAJECTORY:
    LOG_DEBUG("Stoping trajectory");
    this->ctrl_->stopMotionJob(JOB_NAME);
    rtn = true;
    break;
    
  default:
    LOG_ERROR("Adding point: %d to trajectory", jMsg.point_.getSequence());
    this->traj_.addPoint(jMsg.point_); 
    break;
  }
  
  // Send ack
  if (CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    if (rtn)
    {
        jMsg.toReply(reply, ReplyTypes::SUCCESS);
    }
    else
    {
        jMsg.toReply(reply, ReplyTypes::FAILURE);
    }
    
    if(this->getConnection()->sendMsg(reply))
    {
        LOG_INFO("Ack sent");
    }
    else
    {
        LOG_ERROR("Failed to send joint ack");
    }
  }
  
  return rtn;
}

void TrajectoryDownloadHandler::startTrajectory(JointTrajPtMessage & jMsg)
{
    LOG_INFO("Trajectory download initialized, adding first point");
    this->traj_.init();
    this->traj_.addPoint(jMsg.point_); 
}

void TrajectoryDownloadHandler::endTrajectory(JointTrajPtMessage & jMsg)
{

    LOG_INFO("Trajecotry ended, starting job");
    TrajectoryJob job;
    
    // Add end point
    this->traj_.addPoint(jMsg.point_);
    
    // Create, load, and execute motion
    if (job.init(JOB_NAME, this->traj_))
    {
        if(job.toJobString(this->jobBuffer_, JOB_BUFFER_SIZE_))
        {
            if(this->ctrl_->writeJob("", JOB_NAME))
            {
                if(this->ctrl_->loadJob("", JOB_NAME))
                {
                    //this->ctrl_->startMotionJob(JOB_NAME);
                }
                else
                {
                    LOG_ERROR("Failed to load job");
                }
            }
            else
            {
                LOG_ERROR("Failed to write job");
            }
        }
        else
        {
            LOG_ERROR("Failed to convert job to string");
        }
    }
    else
    {
        LOG_ERROR("Failed to intialize job trajectory");
    }
}

}//namespace trajectory_download_handler
}//namespace motoman

