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

#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

#ifdef ROS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"

#endif

#ifdef MOTOPLUS
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"

#endif

namespace industrial
{
namespace robot_status
{


/**
   * \brief Enumeration mirrors industrial_msgs/RobotMode definition
   *
   */
namespace RobotModes
{
enum RobotMode
{
  UNKNOWN = -1,

  MANUAL = 1,
  AUTO = 2,

  CUSTOM_MODE_BEGIN = 100
};
}
typedef RobotModes::RobotMode RobotMode;


/**
   * \brief Enumeration mirrors industrial_msgs/TriState definition
   *
   */
namespace TriStates
{
enum TriState
{
  UNKNOWN = -1,

  TRUE = 1,
  ON = 1,
  ENABLED = 1,
  HIGH = 1,

  FALSE = 0,
  OFF = 0,
  DISABLED = 0,
  LOW = 0
};
}
typedef TriStates::TriState TriState;

/**
 * \brief Class encapsulated robot status data.  The robot status data is
 * meant to mirror the industrial_msgs/RobotStatus message.
 *
 *
 * The byte representation of a robot status is as follows (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   mode                (industrial::shared_types::shared_int)    4  bytes
 *   e_stopped           (industrial::shared_types::shared_int)    4  bytes
 *   drives_powered      (industrial::shared_types::shared_int)    4  bytes
 *   motion_possible     (industrial::shared_types::shared_int)    4  bytes
 *   in_motion           (industrial::shared_types::shared_int)    4  bytes
 *   in_error            (industrial::shared_types::shared_int)    4  bytes
 *   error_code          (industrial::shared_types::shared_int)    4  bytes
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class RobotStatus : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
	RobotStatus(void);
  /**
   * \brief Destructor
   *
   */
  ~RobotStatus(void);

  /**
   * \brief Initializes an empty robot status
   *
   */
  void init();

  industrial::shared_types::shared_int getDrivesPowered() const
      {
          return drives_powered_;
      }

      industrial::shared_types::shared_int getErrorCode() const
      {
          return error_code_;
      }

      industrial::shared_types::shared_int getInError() const
      {
          return in_error_;
      }

      industrial::shared_types::shared_int getInMotion() const
      {
          return in_motion_;
      }

      industrial::shared_types::shared_int getMode() const
      {
          return mode_;
      }

      industrial::shared_types::shared_int getMotionPossible() const
      {
          return motion_possible_;
      }

      void setDrivesPowered(industrial::shared_types::shared_int drives_powered)
      {
          this->drives_powered_ = drives_powered;
      }

      void setErrorCode(industrial::shared_types::shared_int error_code)
      {
          this->error_code_ = error_code;
      }

      void setInError(industrial::shared_types::shared_int in_error)
      {
          this->in_error_ = in_error;
      }

      void setInMotion(industrial::shared_types::shared_int in_motion)
      {
          this->in_motion_ = in_motion;
      }

      void setMode(industrial::shared_types::shared_int mode)
      {
          this->mode_ = mode;
      }

      void setMotionPossible(industrial::shared_types::shared_int motion_possible)
      {
          this->motion_possible_ = motion_possible;
      }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(RobotStatus &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(RobotStatus &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 7 * sizeof(industrial::shared_types::shared_int);
  }

private:

  /**
   * \brief Operating mode (see RobotModes::RobotMode)
   */
  industrial::shared_types::shared_int mode_;

  /**
     * \brief Drive power state (see TriStates::TriState)
     */
    industrial::shared_types::shared_int drives_powered_;

    /**
       * \brief motion possible state (see TriStates::TriState)
       */
      industrial::shared_types::shared_int motion_possible_;

      /**
         * \brief in motion state (see TriStates::TriState)
         */
        industrial::shared_types::shared_int in_motion_;

        /**
           * \brief in error state (see TriStates::TriState)
           */
          industrial::shared_types::shared_int in_error_;

          /**
             * \brief error code
             */
            industrial::shared_types::shared_int error_code_;


};

}
}

#endif /* JOINT_TRAJ_PT_H */
