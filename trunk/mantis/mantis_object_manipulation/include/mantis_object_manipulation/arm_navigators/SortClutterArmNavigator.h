/*
 * SingulateClutterArmNavigator.h
 *
 *  Created on: Dec 24, 2012
 *      Author: coky
 */

#ifndef SORTCLUTTERARMNAVIGATOR_H_
#define SORTCLUTTERARMNAVIGATOR_H_

#include <mantis_object_manipulation/arm_navigators/SpherePickingRobotNavigator.h>
#include <mantis_object_manipulation/arm_navigators/AutomatedPickerRobotNavigator.h>
#include <mantis_object_manipulation/utils/Utilities.h>
#include <mantis_object_manipulation/ArmHandshaking.h>
#include <object_manipulation_tools/manipulation_utils/Utilities.h>

static const int SINGULATED_PICK_ZONE_INDEX = 0;
static const int CLUTTERED_PICK_ZONE_INDEX = 1;
static const int SORTED_PICK_ZONE_INDEX = 1;
static const std::string CLUTTER_DROPOFF_NAMESPACE = "cluttered_dropoff";
static const std::string SINGULATED_DROPOFF_NAMESPACE = "singulated_dropoff";
static const std::string JOINT_HOME_POSITION_NAMESPACE = "joint_home_position";
static const std::string HANDSHAKING_SERVICE_NAME = "arm_handshaking";

class SortClutterArmNavigator: public AutomatedPickerRobotNavigator
{
public:
	SortClutterArmNavigator();
	virtual ~SortClutterArmNavigator();
	virtual void run();

protected:
	// superclass methods
	virtual void setup();
	virtual void fetchParameters(std::string nameSpace="");
	virtual bool performSegmentation();
	virtual bool moveArmThroughPickSequence();
	virtual bool moveArmThroughPlaceSequence();
	virtual bool performPlaceGraspPlanning(); // place pose grasp planning that uses recognition data
	virtual bool performPlaceGraspPlanning(GoalLocation &goal);

	// subclass methods
	bool performGraspPlanningForSorting();
	bool performGraspPlanningForClutter();
	bool performGraspPlanningForSingulation();
	void clearResultsFromLastSrvCall();
	bool moveArmThroughPickPlaceSequence();

	// service callbacks
	bool armHandshakingSrvCallback(mantis_object_manipulation::ArmHandshaking::Request& req,
			mantis_object_manipulation::ArmHandshaking::Response& res);

protected: // members
	mantis_object_manipulation::ArmHandshaking handshaking_data_;
	ros::ServiceServer handshaking_server_;

	std::string clutter_dropoff_ns_;
	std::string singulated_dropoff_ns_;

	GoalLocation clutter_dropoff_location_;
	GoalLocation singulated_dropoff_location_;
};

#endif /* SINGULATECLUTTERARMNAVIGATOR_H_ */
