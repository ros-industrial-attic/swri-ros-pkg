/*
 * SingulateClutterArmNavigator.cpp
 *
 *  Created on: Dec 24, 2012
 *      Author: coky
 */

#include <mantis_object_manipulation/arm_navigators/SortClutterArmNavigator.h>
#include <mantis_perception/mantis_recognition.h>

typedef mantis_object_manipulation::ArmHandshaking::Response HandshakingResp;

// global variables
static const double BOUNDING_SPHERE_RADIUS = 0.01f;

SortClutterArmNavigator::SortClutterArmNavigator()
:AutomatedPickerRobotNavigator()
{
	JOINT_CONFIGURATIONS_NAMESPACE = NODE_NAME + "/" + JOINT_HOME_POSITION_NAMESPACE;
	clutter_dropoff_ns_ = NODE_NAME + "/" + CLUTTER_DROPOFF_NAMESPACE;
	singulated_dropoff_ns_= NODE_NAME + "/" + SINGULATED_DROPOFF_NAMESPACE;
}

SortClutterArmNavigator::~SortClutterArmNavigator() {
	// TODO Auto-generated destructor stub
}

void SortClutterArmNavigator::fetchParameters(std::string nameSpace)
{
	AutomatedPickerRobotNavigator::fetchParameters(nameSpace);
	singulated_dropoff_location_.fetchParameters(singulated_dropoff_ns_);
	clutter_dropoff_location_.fetchParameters(clutter_dropoff_ns_);
}

void SortClutterArmNavigator::clearResultsFromLastSrvCall()
{
	recognized_models_.clear();
	recognized_obj_pose_map_.clear();
	grasp_pickup_goal_.target.potential_models.clear();
	grasp_candidates_.clear();
	handshaking_data_.response.error_code = HandshakingResp::OK;
}

void SortClutterArmNavigator::run()
{
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	srand(time(NULL));

	setup();// full setup

	if(!moveArmToSide())
	{
		ROS_WARN_STREAM(NODE_NAME << ": Side moved failed");
	}

	while(ros::ok())
	{
		ros::Duration(1.0f).sleep();
	}
}

void SortClutterArmNavigator::setup()
{
	ros::NodeHandle nh;
	AutomatedPickerRobotNavigator::setup();
	handshaking_server_ = nh.advertiseService(HANDSHAKING_SERVICE_NAME,
			&SortClutterArmNavigator::armHandshakingSrvCallback,this);
}

bool SortClutterArmNavigator::moveArmThroughPlaceSequence()
{
	using namespace mantis_object_manipulation;
	bool success = RobotNavigator::moveArmThroughPlaceSequence();
	if(success)
	{
		ROS_INFO_STREAM(NODE_NAME<<": Grasp place move succeeded");
		updateMarkerArrayMsg();
	}
	else
	{
		ROS_ERROR_STREAM(NODE_NAME<<"Grasp place move failed, aborting");
		handshaking_data_.response.error_code = mantis_object_manipulation::ArmHandshaking::Response::PLACE_ERROR;
	}
	return success;
}

bool SortClutterArmNavigator::moveArmThroughPickSequence()
{
	using namespace mantis_object_manipulation;

	bool success = AutomatedPickerRobotNavigator::moveArmThroughPickSequence();
	if(success)
	{
		ROS_INFO_STREAM(NODE_NAME<<": Grasp place move succeeded");
	}
	else
	{
		ROS_ERROR_STREAM(NODE_NAME<<"Grasp place move failed, aborting");
		handshaking_data_.response.error_code = ArmHandshaking::Response::PICK_ERROR;
	}
	return success;
}

bool SortClutterArmNavigator::armHandshakingSrvCallback(mantis_object_manipulation::ArmHandshaking::Request& req,
			mantis_object_manipulation::ArmHandshaking::Response& res)
{
	using namespace mantis_object_manipulation;

	bool success = true;
	handshaking_data_.request = req;

	// initializing response
	res.completed = true;
	res.error_code = HandshakingResp::OK;

	// clearing variables
	clearResultsFromLastSrvCall();

	// processing command
	switch(handshaking_data_.request.command)
	{
	case ArmHandshaking::Request::SINGULATE_CLUTTER:

		zone_selector_.goToPickZone(CLUTTERED_PICK_ZONE_INDEX);

		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
		if(!performSegmentation())
		{
			ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << " Segmentation stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage started");
		if(!performGraspPlanningForSingulation())
		{
			ROS_ERROR_STREAM(NODE_NAME<<": Grasp Planning stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage completed");

		success = moveArmThroughPickPlaceSequence();

		break;

	case ArmHandshaking::Request::SINGULATE_SORTED:

		zone_selector_.goToPickZone(SORTED_PICK_ZONE_INDEX);

		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
		if(!performSegmentation())
		{
			ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << " Segmentation stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage started");
		if(!performRecognition())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Recognition stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage started");
		if(!performGraspPlanningForSingulation())
		{
			ROS_ERROR_STREAM(NODE_NAME<<": Grasp Planning stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage completed");

		success = moveArmThroughPickPlaceSequence();

		break;

	case ArmHandshaking::Request::SORT:

		zone_selector_.goToPickZone(SINGULATED_PICK_ZONE_INDEX);

		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
		if(!performSegmentation())
		{
			ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << " Segmentation stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage started");
		if(!performRecognition())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Recognition stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage started");
		if(!performGraspPlanningForSorting())
		{
			ROS_ERROR_STREAM(NODE_NAME<<": Grasp Planning stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage completed");

		success = moveArmThroughPickPlaceSequence();

		break;

	case ArmHandshaking::Request::CLUTTER:

		zone_selector_.goToPickZone(SINGULATED_PICK_ZONE_INDEX);

		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
		if(!performSegmentation())
		{
			ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << " Segmentation stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage started");
		if(!performRecognition())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Recognition stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage started");
		if(!performGraspPlanningForClutter())
		{
			ROS_ERROR_STREAM(NODE_NAME<<": Grasp Planning stage failed");
			success = false;
			break;
		}
		ROS_INFO_STREAM(NODE_NAME << ": Grasp Planning stage completed");

		success = moveArmThroughPickPlaceSequence();

		break;

	case ArmHandshaking::Request::MOVE_HOME:

		if(!moveArmToSide())
		{
			handshaking_data_.response.error_code = ArmHandshaking::Response::FINAL_MOVE_HOME_ERROR;
			success = false;
		}

		break;

	case ArmHandshaking::Request::WAIT:

		break;

	case ArmHandshaking::Request::TERMINATE:

		ros::shutdown();
		return true;

	default:

		success = false;
		handshaking_data_.response.error_code = HandshakingResp::UNKNOWN_COMMAND_REQUEST_ERROR;
		break;
	}

	res.completed = success;
	res.error_code = handshaking_data_.response.error_code;


	return true;
}

bool SortClutterArmNavigator::moveArmThroughPickPlaceSequence()
{
	// beginning movement
	ROS_INFO_STREAM(NODE_NAME + ": Grasp Pick stage started");
	if(!moveArmThroughPickSequence())
	{
	  ROS_WARN_STREAM(NODE_NAME << ": Grasp Pick stage failed");
	  moveArmToSide();
	  return false;
	}
	else
	{
		ROS_INFO_STREAM(NODE_NAME << ": Grasp Pick stage completed");
	}

	ROS_INFO_STREAM(NODE_NAME + ": Grasp Place stage started");
	if(!moveArmThroughPlaceSequence())
	{
		ROS_WARN_STREAM(NODE_NAME << ": Grasp Place stage failed");
		moveArmToSide();
		return false;
	}
	else
	{
		ROS_INFO_STREAM(NODE_NAME << ": Grasp Place stage completed");
	}

	if(!moveArmToSide())
	{
		ROS_WARN_STREAM(NODE_NAME << ": Side moved failed");
		handshaking_data_.response.error_code = HandshakingResp::FINAL_MOVE_HOME_ERROR;
		return false;
	}

	return true;
}

bool SortClutterArmNavigator::performSegmentation()
{
	using namespace mantis_object_manipulation;

	// performing segmentation
	if(!RobotNavigator::performSegmentation())
	{
		if(segmentation_results_.result != segmentation_results_.SUCCESS)
		{
			handshaking_data_.response.error_code = ArmHandshaking::Response::PERCEPTION_ERROR;
			return false;
		}

		if(segmented_clusters_.empty())
		{
			handshaking_data_.response.error_code = ArmHandshaking::Response::NO_CLUSTERS_FOUND;
			return false;
		}
	}

	// clearing obstacle clusters from zone
	zone_selector_.clearObstableClusters();

	// check if at least one cluster is located in pick zone
	std::vector<int> inZone;
	bool clustersFound = zone_selector_.isInPickZone(segmented_clusters_,inZone);
	if(!clustersFound)
	{
		ROS_WARN_STREAM(NODE_NAME<<": Neither cluster was found in pick zone, canceling");
		handshaking_data_.response.error_code = ArmHandshaking::Response::NO_CLUSTERS_FOUND;
		return false;
	}
	else
	{
		ROS_INFO_STREAM(NODE_NAME<<": A total of "<<inZone.size()<<" were found in pick zone");

		// finding clusters outside of pick zone to be used as obstacles
		std::vector<sensor_msgs::PointCloud> tempArray;
		for(std::size_t i = 0;i < segmented_clusters_.size();i++)
		{
			if(std::find(inZone.begin(),inZone.end(),i) == inZone.end())
			{
				// not in pick zone
				tempArray.push_back(segmented_clusters_[i]);
			}
		}

		// adding clusters
		if(tempArray.size() > 0)
		{
			zone_selector_.addObstacleClusters(tempArray);
			tempArray.clear();
		}

		// retaining only cluster in pick zone
		for(std::size_t i = 0;i < inZone.size();i++)
		{
			tempArray.push_back(segmented_clusters_[inZone[i]]);
		}
		segmented_clusters_.assign(tempArray.begin(),tempArray.end());
	}

	updateMarkerArrayMsg();
	return true;
}

bool SortClutterArmNavigator::performPlaceGraspPlanning()
{
	using namespace mantis_object_manipulation;

	// Finding location in place zone for recognized object
	ROS_WARN_STREAM(NODE_NAME<<": finding place location for bb box of side length "<<attached_obj_bb_side_);

	updateMarkerArrayMsg();
	tf::Vector3 objSize = tf::Vector3(attached_obj_bb_side_,attached_obj_bb_side_,attached_obj_bb_side_);
	PickPlaceZoneSelector::ObjectDetails objDetails(tf::Transform::getIdentity(),objSize,
			recognition_result_.model_id,recognition_result_.label);
	zone_selector_.setNextObjectDetails(objDetails);

	candidate_place_poses_.clear();
	if(!zone_selector_.generateNextLocationCandidates(candidate_place_poses_))
	{
		ROS_WARN_STREAM(NODE_NAME<<": Couldn't find available location for object, cancelling");
		// no more locations available, swapping zones
		updateMarkerArrayMsg();
		return false;
	}

	//	updating grasp place goal data
	grasp_place_goal_.arm_name = arm_group_name_;
	grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
	grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];

	// finding valid grasp place sequence
	bool found_valid = false;
	geometry_msgs::Pose valid_grasp_place_pose;
	geometry_msgs::Pose tcp_in_objct_pose;
	std::vector<object_manipulation_msgs::Grasp> valid_grasps; // will keep only valid grasp pick sequence which grasp yields a valid place sequence;
	std::vector<object_manipulator::GraspExecutionInfo> valid_pick_sequence;
	std::vector<object_manipulator::PlaceExecutionInfo> valid_place_sequence;

	// finding pose of wrist relative to object
	updateChangesToPlanningScene();
	tf::StampedTransform wrist_in_tcp_tf = tf::StampedTransform(), wrist_in_obj_tf = tf::StampedTransform();
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);
	for(std::size_t i = 0; i < grasp_candidates_.size(); i++)
	{

		// storing tcp to object pose in grasp place goal
		tf::poseMsgToTF(grasp_candidates_[i].grasp_pose,wrist_in_obj_tf);
		tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),tcp_in_objct_pose);
		manipulation_utils::rectifyPoseZDirection(tcp_in_objct_pose,
				PLACE_RECTIFICATION_TF,grasp_place_goal_.grasp.grasp_pose);

		if(createPlaceMoveSequence(grasp_place_goal_,candidate_place_poses_,valid_place_sequence))
		{
			if(!found_valid)
			{
				// storing first valid
				grasp_place_sequence_.assign(valid_place_sequence.begin(),valid_place_sequence.end());
				valid_grasp_place_pose = grasp_place_goal_.grasp.grasp_pose;
			}

			found_valid = true;
			valid_grasps.push_back(grasp_candidates_[i]);
			valid_pick_sequence.push_back(grasp_pick_sequence_[i]);
		}
	}

	if(!found_valid)
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp place sequence");
		return false;
	}
	else
	{
		// storing valid pick/place data
		grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());
		grasp_pick_sequence_.assign(valid_pick_sequence.begin(),valid_pick_sequence.end());
		grasp_place_goal_.grasp.grasp_pose = valid_grasp_place_pose;

	}

	return found_valid;
}

bool SortClutterArmNavigator::performPlaceGraspPlanning(GoalLocation &goal)
{
	using namespace mantis_object_manipulation;

	// Check that goal location is valid
	candidate_place_poses_.clear();
	goal.generateNextLocationCandidates(candidate_place_poses_);

	//	updating grasp place goal data
	grasp_place_goal_.arm_name = arm_group_name_;
	grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
	grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];

	// finding valid grasp place sequence
	bool found_valid = false;
	geometry_msgs::Pose valid_grasp_place_pose;
	geometry_msgs::Pose tcp_in_objct_pose;
	std::vector<object_manipulation_msgs::Grasp> valid_grasps; // will keep only valid grasp pick sequence which grasp yields a valid place sequence;
	std::vector<object_manipulator::GraspExecutionInfo> valid_pick_sequence;
	std::vector<object_manipulator::PlaceExecutionInfo> valid_place_sequence;

	// finding pose of wrist relative to object
	updateChangesToPlanningScene();
	tf::StampedTransform wrist_in_tcp_tf = tf::StampedTransform(), wrist_in_obj_tf = tf::StampedTransform();
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);
	for(std::size_t i = 0; i < grasp_candidates_.size(); i++)
	{
		// storing tcp to object pose in grasp place goal
		tf::poseMsgToTF(grasp_candidates_[i].grasp_pose,wrist_in_obj_tf);
		tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),tcp_in_objct_pose);
		manipulation_utils::rectifyPoseZDirection(tcp_in_objct_pose,
				PLACE_RECTIFICATION_TF,grasp_place_goal_.grasp.grasp_pose);

		if(createPlaceMoveSequence(grasp_place_goal_,candidate_place_poses_,valid_place_sequence))
		{
			if(!found_valid)
			{
				// storing first valid
				grasp_place_sequence_.assign(valid_place_sequence.begin(),valid_place_sequence.end());
				valid_grasp_place_pose = grasp_place_goal_.grasp.grasp_pose;
			}

			found_valid = true;
			valid_grasps.push_back(grasp_candidates_[i]);
			valid_pick_sequence.push_back(grasp_pick_sequence_[i]);
		}
	}

	if(!found_valid)
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp place sequence");
		return false;
	}
	else
	{
		// storing valid pick/place data
		grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());
		grasp_pick_sequence_.assign(valid_pick_sequence.begin(),valid_pick_sequence.end());
		grasp_place_goal_.grasp.grasp_pose = valid_grasp_place_pose;

	}

	return found_valid;
}

bool SortClutterArmNavigator::performGraspPlanningForSorting()
{
	using namespace mantis_object_manipulation;

	// call grasp planning method that uses recognition results
	if(!AutomatedPickerRobotNavigator::performPickGraspPlanning())
	{
		handshaking_data_.response.error_code = ArmHandshaking::Response::GRASP_PLANNING_ERROR;
		return false;
	}

	if(!performPlaceGraspPlanning())
	{
		handshaking_data_.response.error_code = ArmHandshaking::Response::GRASP_PLANNING_ERROR;
		return false;
	}

	return true;
}

bool SortClutterArmNavigator::performGraspPlanningForClutter()
{
	using namespace mantis_object_manipulation;

	// call grasp planning method that uses recognition results
	if(!AutomatedPickerRobotNavigator::performPickGraspPlanning())
	{
		handshaking_data_.response.error_code = ArmHandshaking::Response::GRASP_PLANNING_ERROR;
		return false;
	}

	// call grasp place planning in clutter zone
	if(!performPlaceGraspPlanning(clutter_dropoff_location_))
	{
		handshaking_data_.response.error_code = ArmHandshaking::Response::GRASP_PLANNING_ERROR;
		return false;
	}

	return true;
}

bool SortClutterArmNavigator::performGraspPlanningForSingulation()
{
	using namespace mantis_object_manipulation;

	bool success = false;
	// checking if there are recognition results
	if(recognized_models_.empty())
	{
		// no recognition data then it must be moving objects from clutter zone, creating dummy recognized model
		arm_navigation_msgs::CollisionObject obj;
		manipulation_utils::createBoundingSphereCollisionModel(segmented_clusters_[0],BOUNDING_SPHERE_RADIUS,obj);
		obj.id = makeCollisionObjectNameFromModelId(0);
		obj.header.frame_id = cm_.getWorldFrameId();
		obj.padding = 0;
		addDetectedObjectToLocalPlanningScene(obj);

		household_objects_database_msgs::DatabaseModelPoseList models;
		household_objects_database_msgs::DatabaseModelPose model;
		model.model_id = 0;
		model.confidence = 1.0f;
		model.detector_name = "dummy_model";
		model.pose.pose = obj.poses[0];
		models.model_list.push_back(model);
		recognized_models_.push_back(models);
		recognized_obj_pose_map_[std::string(obj.id)] = model.pose;

		// now call base grasp planning method
		success = RobotNavigator::performPickGraspPlanning();

	}
	else
	{
		// recognition data available then it must be moving objects from sorted zone
		success = AutomatedPickerRobotNavigator::performPickGraspPlanning();
	}

	// call grasp place planning in clutter zone
	if(success)
	{
		success = performPlaceGraspPlanning(singulated_dropoff_location_);
	}


	if(!success)
	{
		handshaking_data_.response.error_code = ArmHandshaking::Response::GRASP_PLANNING_ERROR;
	}

	return success;
}
