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

// defaults and constants
static const int DF_SINGULATED_ZONE_INDEX = 0;
static const int DF_CLUTTERED_ZONE_INDEX = 1;
static const int DF_SORTED_ZONE_INDEX = 1;
static const std::string DF_SINGULATION_SEGMENTATION_SRV = "singulation_segmentation";
static const std::string CLUTTER_DROPOFF_NAMESPACE = "cluttered_dropoff";
static const std::string SINGULATED_DROPOFF_NAMESPACE = "singulated_dropoff";
static const std::string JOINT_HOME_POSITION_NAMESPACE = "joint_home_position";
static const std::string HANDSHAKING_SERVICE_NAME = "arm_handshaking";

// param names
static const std::string PARAM_SINGULATION_SEGMENTATION_SRV= "singulation_segmentation_service_name";
static const std::string PARAM_SINGULATION_ZONE_INDEX = "singulation_zone_index";
static const std::string PARAM_CLUTTERED_ZONE_INDEX = "cluttered_zone_index";
static const std::string PARAM_SORTED_ZONE_INDEX = "sorted_zone_index";

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
	virtual bool performPickGraspPlanning(); // grasp planning on cluster data
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

	bool armHandshakingTaskHandler(mantis_object_manipulation::ArmHandshaking::Request& req,
			mantis_object_manipulation::ArmHandshaking::Response& res);



protected:
	// service members
	mantis_object_manipulation::ArmHandshaking handshaking_data_;
	ros::ServiceServer handshaking_server_;

	// ros connections
	ros::ServiceClient singulation_segmentation_client_;

	// ros parameters
	std::string clutter_dropoff_ns_;
	std::string singulated_dropoff_ns_;

	GoalLocation clutter_dropoff_location_;
	GoalLocation singulated_dropoff_location_;

	std::string singulation_segmentation_srv_;
	int singulation_zone_index_;
	int cluttered_zone_index_;
	int sorted_zone_index_;

};

#endif /* SINGULATECLUTTERARMNAVIGATOR_H_ */
