/*
 * PlaceSequenceValidator.h
 *
 *  Created on: Oct 10, 2012
 */

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

#ifndef PLACE_SEQUENCE_VALIDATOR_H_
#define PLACE_SEQUENCE_VALIDATOR_H_

#include <object_manipulator/place_execution/place_tester_fast.h>
#include <object_manipulator/place_execution/descend_retreat_place.h>
#include <object_manipulator/tools/hand_description.h>
#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>
#include <tf/transform_datatypes.h>

class PlaceSequenceValidator : public object_manipulator::PlaceTesterFast
{
public:
	PlaceSequenceValidator();

	PlaceSequenceValidator(planning_environment::CollisionModels* cm = NULL,
			  const std::string& plugin_name="SIA20D_Mesh_manipulator_kinematics/IKFastKinematicsPlugin");

	virtual ~PlaceSequenceValidator();

	void setTcpToWristTransform(const tf::Transform &tcpTransform); // pose of the tcp relative to the wrist;

	std::map<std::string,arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*>& getIkSolverMap()
	{
		return ik_solver_map_;
	}

	void testPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
            const std::vector<geometry_msgs::PoseStamped> &place_locations,
            std::vector<object_manipulator::PlaceExecutionInfo> &execution_info,
            bool return_on_first_hit);


protected:

	tf::Transform _TcpToWristTransform;
};

#endif /* PLACE_SEQUENCE_VALIDATOR_H_ */
