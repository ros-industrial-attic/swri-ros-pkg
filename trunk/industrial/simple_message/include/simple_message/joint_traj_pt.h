﻿/*
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

#ifndef JOINT_TRAJ_PT_H
#define JOINT_TRAJ_PT_H

#ifdef ROS
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"

#endif

#ifdef MOTOPLUS
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"

#endif



namespace industrial
{
namespace joint_traj_pt
{

/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory.  This point differs from the ROS
 * trajectory point in that is specifies the joint velocity in a robot standard
 * way (as a single value).
 */
//* JointTrajPt
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPt : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
	JointTrajPt(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPt(void);

  /**
   * \brief Initializes a empty joint trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory point
   *
   */
  void init(industrial::shared_types::shared_int sequence,
		  industrial::joint_data::JointData position,
		  industrial::shared_types::shared_real velocity);

  /**
   * \brief Sets joint position data
   *
   * \param joint position data
   */
  void setJointPosition(industrial::joint_data::JointData &position)
  {
	  this->joint_position_.copyFrom(position);
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param joint position dest
   */
  void getJointPosition(industrial::joint_data::JointData &dest)
  {
	  dest.copyFrom(this->joint_position_);
  }

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
	  this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
	  return this->sequence_;
  }

  /**
     * \brief Sets joint trajectory point velocity
     *
     * \param velocity value
     */
    void setVelocity(industrial::shared_types::shared_real velocity)
    {
  	  this->velocity_ = velocity;
    }

    /**
     * \brief Returns joint trajectory point velocity
     *
     * \return joint trajectory velocity
     */
    industrial::shared_types::shared_real getVelocity()
    {
  	  return this->velocity_;
    }


  
  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointTrajPt &src);

  /**
     * \brief == operator implementation
     *
     * \return true if equal
     */
  bool operator==(JointTrajPt &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
	  return sizeof(industrial::shared_types::shared_real) +
			  sizeof(industrial::shared_types::shared_int) +
			  this->joint_position_.byteLength();
  }

private:

  /**
   * \brief joint point positional data
   */
  industrial::joint_data::JointData joint_position_;
  /**
   * \brief joint point velocity
   */
  industrial::shared_types::shared_real velocity_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

};

}
}

#endif /* JOINT_TRAJ_PT_H */
