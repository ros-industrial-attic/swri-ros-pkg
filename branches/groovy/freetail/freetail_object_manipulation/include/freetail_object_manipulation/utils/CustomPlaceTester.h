/*
 * CustomPlaceTester.h
 *
 *  Created on: Sep 14, 2012
 *      Author: jnicho
 */

#ifndef CUSTOMPLACETESTER_H_
#define CUSTOMPLACETESTER_H_

#include <object_manipulator/place_execution/place_tester_fast.h>
#include <object_manipulator/place_execution/descend_retreat_place.h>
#include <object_manipulator/tools/hand_description.h>
#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>

#include <tf/transform_datatypes.h>

class CustomPlaceTester : public object_manipulator::PlaceTesterFast
{
public:
	CustomPlaceTester();

	CustomPlaceTester(planning_environment::CollisionModels* cm = NULL,
			  const std::string& plugin_name="pr2_arm_kinematics/PR2ArmKinematicsPlugin");

	virtual ~CustomPlaceTester();

	void setTcpToWristTransform(const tf::Transform &tcpTransform); // pose of the tcp relative to the wrist;

	void testPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
            const std::vector<geometry_msgs::PoseStamped> &place_locations,
            std::vector<object_manipulator::PlaceExecutionInfo> &execution_info,
            bool return_on_first_hit);

protected:



	tf::Transform _TcpToWristTransform;
};

#endif /* CUSTOMPLACETESTER_H_ */
