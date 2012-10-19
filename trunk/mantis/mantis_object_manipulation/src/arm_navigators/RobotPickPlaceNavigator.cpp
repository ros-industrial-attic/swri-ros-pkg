/*
 * RobotPickPlaceNavigator.cpp
 *
 *  Created on: Oct 8, 2012
 *      Author: jnicho
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

#include <mantis_object_manipulation/arm_navigators/RobotPickPlaceNavigator.h>
#include <boost/foreach.hpp>

using namespace trajectory_execution_monitor;

// node name
std::string NODE_NAME = "robot_pick_place";

// name spaces
std::string NAVIGATOR_NAMESPACE = "navigator";
std::string SEGMENTATION_NAMESPACE = "segmentation";
std::string GOAL_NAMESPACE = "goal";
std::string JOINT_CONFIGURATIONS_NAMESPACE = "joints";

// marker map
std::map<std::string,visualization_msgs::Marker> MarkerMap;
const std::string MARKER_ATTACHED_OBJ = "attached_obj";
const std::string MARKER_SEGMENTED_OBJ = "segmented_obj";


void RobotPickPlaceNavigator::fetchParameters(std::string nameSpace)
{
	ros::NodeHandle nh;
	ros::param::param(nameSpace + "/" + PARAM_NAME_ARM_GROUP,arm_group_name_,DEFAULT_ARM_GROUP);
	ros::param::param(nameSpace + "/" + PARAM_NAME_GRIPPER_GROUP,gripper_group_name_,DEFAULT_GRIPPER_GROUP);
	ros::param::param(nameSpace + "/" + PARAM_NAME_WRIST_LINK,wrist_link_name_,DEFAULT_WRIST_LINK);
	ros::param::param(nameSpace + "/" + PARAM_NAME_GRIPPER_LINK,gripper_link_name_,DEFAULT_GRIPPER_LINK);
	ros::param::param(nameSpace + "/" + PARAM_NAME_GRASP_ACTION_SERVICE,grasp_action_service_,DEFAULT_GRASP_ACTION_SERVICE );
	ros::param::param(nameSpace + "/" + PARAM_NAME_TRAJECTORY_ACTION_SERVICE,trajectory_action_service_,DEFAULT_TRAJECTORY_ACTION_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_PATH_PLANNER_SERVICE,path_planner_service_,DEFAULT_PATH_PLANNER_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_TRAJECTORY_FILTER_SERVICE,trajectory_filter_service_,DEFAULT_TRAJECTORY_FILTER_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_SEGMENTATION_SERVICE,segmentation_service_,DEFAULT_SEGMENTATION_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_RECOGNITION_SERVICE,recognition_service_,DEFAULT_RECOGNITION_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_GRASP_PLANNING_SERVICE,grasp_planning_service_,DEFAULT_GRASP_PLANNING_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_MESH_DATABASE_SERVICE,mesh_database_service_,DEFAULT_MESH_DATABASE_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_MODEL_DATABASE_SERVICE,model_database_service_,DEFAULT_MODEL_DATABASE_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_PLANNING_SCENE_SERVICE,planning_scene_service_,DEFAULT_PLANNING_SCENE_SERVICE);
	ros::param::param(nameSpace + "/" + PARAM_NAME_IK_PLUGING,ik_plugin_name_,DEFAULT_IK_PLUGING);
	ros::param::param(nameSpace + "/" + PARAM_NAME_JOINT_STATES_TOPIC,joint_states_topic_,DEFAULT_JOINT_STATES_TOPIC);
}

RobotPickPlaceNavigator::RobotPickPlaceNavigator(ConfigurationFlags flag)
:configuration_type_(flag),
 cm_("robot_description"),
 current_robot_state_(NULL)
{
	ros::NodeHandle nh;

	// initializing name spaces global strings
	NODE_NAME = ros::this_node::getName();
	NAVIGATOR_NAMESPACE = NODE_NAME + "/" + NAVIGATOR_NAMESPACE;
	GOAL_NAMESPACE = NODE_NAME + "/" + GOAL_NAMESPACE;
	SEGMENTATION_NAMESPACE = NODE_NAME + "/" + SEGMENTATION_NAMESPACE;
	JOINT_CONFIGURATIONS_NAMESPACE = NODE_NAME + "/" + JOINT_CONFIGURATIONS_NAMESPACE;
}

RobotPickPlaceNavigator::~RobotPickPlaceNavigator()
{
	// TODO Auto-generated destructor stub
}

void RobotPickPlaceNavigator::setup()
{
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();

	ROS_INFO_STREAM(NODE_NAME<<": Loading ros parameters");

	// getting ros parametets
	fetchParameters(NAVIGATOR_NAMESPACE);

	ROS_INFO_STREAM(NODE_NAME<<": Setting up execution Monitors");
	// setting up execution monitors
	{
		joint_state_recorder_.reset(new JointStateTrajectoryRecorder(joint_states_topic_));
		arm_controller_handler_.reset(new FollowJointTrajectoryControllerHandler(arm_group_name_,trajectory_action_service_));
		gripper_controller_handler_.reset(new GraspPoseControllerHandler(gripper_group_name_,grasp_action_service_));

		trajectory_execution_monitor_.addTrajectoryRecorder(joint_state_recorder_);
		trajectory_execution_monitor_.addTrajectoryControllerHandler(arm_controller_handler_);
		trajectory_execution_monitor_.addTrajectoryControllerHandler(gripper_controller_handler_);
	}

	ROS_INFO_STREAM(NODE_NAME<<": Setting up Service Clients");
    // setting up service clients, this is configuration specific
	{
		switch(configuration_type_)
		{
		case SETUP_SPHERE_PICK_PLACE:

			seg_srv_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(segmentation_service_, true);
			break;

		case SETUP_FULL:

			seg_srv_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(segmentation_service_, true);
			rec_srv_ = nh.serviceClient<tabletop_object_detector::TabletopObjectRecognition>(
					recognition_service_, true);
			object_database_model_mesh_client_ = nh.serviceClient<household_objects_database_msgs::GetModelMesh>(
					mesh_database_service_, true);
			object_database_model_description_client_ = nh.serviceClient<household_objects_database_msgs::GetModelDescription>(
					model_database_service_, true);
			break;

		case SETUP_OTHER:

			ROS_ERROR_STREAM(NODE_NAME<<": This configuration option is not supported");
			exit(0);
		}

		grasp_planning_client = nh.serviceClient<object_manipulation_msgs::GraspPlanning>(grasp_planning_service_, true);
		planning_service_client_ = nh.serviceClient<arm_navigation_msgs::GetMotionPlan>(path_planner_service_);

		// arm trajectory filter service
		trajectory_filter_service_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(trajectory_filter_service_);
		ROS_INFO_STREAM(NODE_NAME<<": Waiting for trajectory filter service");
		trajectory_filter_service_client_.waitForExistence();
		ROS_INFO_STREAM(NODE_NAME<<": Trajectory filter service connected");

		// planing scene
		ROS_INFO_STREAM(NODE_NAME <<": Waiting for " + planning_scene_service_ + " service");
		ros::service::waitForService(planning_scene_service_);
		set_planning_scene_diff_client_ = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(planning_scene_service_);
	}

	// will use grasp execution client to request pre-grasp action since the default gripper controller handler
	// ignores this step.
	ROS_INFO_STREAM(NODE_NAME << ": Setting up Service Action Clients");
	{
		grasp_exec_action_client_ =
				boost::make_shared< GraspActionServerClient >(grasp_action_service_,true);
		while(!grasp_exec_action_client_->waitForServer(ros::Duration(0.5)))
		{
			ROS_INFO_STREAM(NODE_NAME << "Waiting for action service "<< grasp_action_service_);
		}
		ROS_INFO_STREAM(NODE_NAME<<" : Connected to action service "<<grasp_action_service_);
	}

	ROS_INFO_STREAM(NODE_NAME<<": Setting up ros publishers");
	{
		// setting up ros publishers
		vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> ("freetail_object", 1);
		vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> ("freetail_object_array", 1);
		attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);

		// setting up timer obj
		_MarkerPubTimer = nh.createTimer(ros::Duration(0.4f),&RobotPickPlaceNavigator::callbackPublishMarkers,this);

		ROS_INFO_STREAM(NODE_NAME<<": Setting up dynamic libraries");
		// others
		grasp_tester_ = new object_manipulator::GraspTesterFast(&cm_, ik_plugin_name_);
		place_tester_ = new PlaceSequenceValidator(&cm_, ik_plugin_name_);
		trajectories_finished_function_ = boost::bind(&RobotPickPlaceNavigator::trajectoriesFinishedCallbackFunction, this, _1);

		ROS_INFO_STREAM(NODE_NAME<<": Finished setup");
	}

	ROS_INFO_STREAM(NODE_NAME<<": Setting up published markers");
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = cm_.getWorldFrameId();
		marker.ns = NODE_NAME;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::DELETE;
		tf::poseTFToMsg(tf::Transform::getIdentity(),marker.pose);

		// adding marker to map
		marker.id = 0; MarkerMap.insert(std::make_pair(MARKER_SEGMENTED_OBJ,marker));
		marker.id = 1; MarkerMap.insert(std::make_pair(MARKER_ATTACHED_OBJ,marker));
	}
}

void RobotPickPlaceNavigator::setupBallPickingDemo()
{
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();

	ROS_INFO_STREAM(NODE_NAME<<": Loading ros parameters");

	// getting ros parametets
	fetchParameters(NAVIGATOR_NAMESPACE);

	ROS_INFO_STREAM(NODE_NAME<<": Setting up execution Monitors");
	// setting up execution monitors
	{
		joint_state_recorder_.reset(new JointStateTrajectoryRecorder(joint_states_topic_));
		arm_controller_handler_.reset(new FollowJointTrajectoryControllerHandler(arm_group_name_,trajectory_action_service_));
		gripper_controller_handler_.reset(new GraspPoseControllerHandler(gripper_group_name_,grasp_action_service_));

		trajectory_execution_monitor_.addTrajectoryRecorder(joint_state_recorder_);
		trajectory_execution_monitor_.addTrajectoryControllerHandler(arm_controller_handler_);
		trajectory_execution_monitor_.addTrajectoryControllerHandler(gripper_controller_handler_);
	}

	ROS_INFO_STREAM(NODE_NAME<<": Setting up Service Clients");
    // setting up service clients
	{
		// segmentation service
		seg_srv_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(segmentation_service_, true);

		// grasp planning
		grasp_planning_client = nh.serviceClient<object_manipulation_msgs::GraspPlanning>(grasp_planning_service_, true);

		// arm path planning service
		planning_service_client_ = nh.serviceClient<arm_navigation_msgs::GetMotionPlan>(path_planner_service_);

		// arm trajectory filter service
		trajectory_filter_service_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(trajectory_filter_service_);
		ROS_INFO_STREAM(NODE_NAME<<": Waiting for trajectory filter service");
		trajectory_filter_service_client_.waitForExistence();
		ROS_INFO_STREAM(NODE_NAME<<": Trajectory filter service connected");

		// planing scene
		ROS_INFO_STREAM(NODE_NAME <<": Waiting for " + planning_scene_service_ + " service");
		ros::service::waitForService(planning_scene_service_);
		set_planning_scene_diff_client_ = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(planning_scene_service_);
	}

	// will use grasp execution client to request pre-grasp action since the default gripper controller handler
	// ignores this step.
	ROS_INFO_STREAM(NODE_NAME << ": Setting up Service Action Clients");
	{
		grasp_exec_action_client_ =
				boost::make_shared< GraspActionServerClient >(grasp_action_service_,true);
		while(!grasp_exec_action_client_->waitForServer(ros::Duration(0.5)))
		{
			ROS_INFO_STREAM(NODE_NAME << "Waiting for action service "<< grasp_action_service_);
		}
		ROS_INFO_STREAM(NODE_NAME<<" : Connected to action service "<<grasp_action_service_);
	}

	ROS_INFO_STREAM(NODE_NAME<<": Setting up ros publishers");
	{
		// setting up ros publishers
		vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> ("freetail_object", 1);
		vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> ("freetail_object_array", 1);
		attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);

		// setting up timer obj
		_MarkerPubTimer = nh.createTimer(ros::Duration(0.4f),&RobotPickPlaceNavigator::callbackPublishMarkers,this);

		ROS_INFO_STREAM(NODE_NAME<<": Setting up dynamic libraries");
		// others
		grasp_tester_ = new object_manipulator::GraspTesterFast(&cm_, ik_plugin_name_);
		place_tester_ = new PlaceSequenceValidator(&cm_, ik_plugin_name_);
		trajectories_finished_function_ = boost::bind(&RobotPickPlaceNavigator::trajectoriesFinishedCallbackFunction, this, _1);

		ROS_INFO_STREAM(NODE_NAME<<": Finished setup");
	}

	ROS_INFO_STREAM(NODE_NAME<<": Setting up published markers");
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = cm_.getWorldFrameId();
		marker.ns = NODE_NAME;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::DELETE;
		tf::poseTFToMsg(tf::Transform::getIdentity(),marker.pose);

		// adding marker to map
		marker.id = 0; MarkerMap.insert(std::make_pair(MARKER_SEGMENTED_OBJ,marker));
		marker.id = 1; MarkerMap.insert(std::make_pair(MARKER_ATTACHED_OBJ,marker));
	}
}

void RobotPickPlaceNavigator::setupRecognitionOnly()
{
	ros::NodeHandle nh;

	ROS_INFO("Loading ros parameters");

	// getting ros parametets
	fetchParameters(NAVIGATOR_NAMESPACE);

	ROS_INFO("Setting up Service Clients");
    // setting up service clients
	{
		rec_srv_ = nh.serviceClient<tabletop_object_detector::TabletopObjectRecognition>(recognition_service_, true);

		ROS_INFO_STREAM("Waiting for " + recognition_service_+ " service");
		while(!ros::service::waitForService(recognition_service_,ros::Duration(4.0f)))
		{
			ROS_INFO_STREAM("Waiting for " + recognition_service_+ " service");
		}
		ROS_INFO_STREAM("Connected to " + recognition_service_+ " service");
	}
}

bool RobotPickPlaceNavigator::trajectoriesFinishedCallbackFunction(TrajectoryExecutionDataVector tedv)
{
	if(tedv.size()==0)
	{
		ROS_ERROR_STREAM(NODE_NAME <<": trajectory finished callback received empty vector");
		return true;
	}

   last_trajectory_execution_data_vector_ = tedv;

   trajectories_succeeded_ = (tedv.back().result_ == SUCCEEDED
                              || tedv.back().result_ == HANDLER_REPORTS_FAILURE_BUT_OK
                              || tedv.back().result_ == HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH);

   ROS_INFO_STREAM(NODE_NAME<<": Trajectories " << tedv.size() << " ok " << trajectories_succeeded_);

   for(unsigned int i = 0; i < tedv.size(); i++)
   {
     ROS_INFO_STREAM("Recorded trajectory " << i << " has " << tedv[i].recorded_trajectory_.points.size());
   }
   execution_completed_.notify_all();
   return true;
 }

void RobotPickPlaceNavigator::revertPlanningScene()
{

  if(current_robot_state_ != NULL)
  {
    cm_.revertPlanningScene(current_robot_state_);
    current_robot_state_ = NULL;
  }
  last_mpr_id_ = 0;
  max_mpr_id_ = 0;
  max_trajectory_id_ = 0;
}

std::vector<std::string> RobotPickPlaceNavigator::getJointNames(const std::string& group) {
  if(cm_.getKinematicModel()->getModelGroup(group) == NULL) {
    std::vector<std::string> names;
    return names;
  }
  return cm_.getKinematicModel()->getModelGroup(group)->getJointModelNames();
}

void RobotPickPlaceNavigator::updateCurrentJointStateToLastTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj)
{
  if(traj.points.size() == 0)
  {
    ROS_ERROR_STREAM(NODE_NAME<<": No points in trajectory for setting current state");
    return;
  }

  std::map<std::string, double> joint_vals;
  for(unsigned int i = 0; i < traj.joint_names.size(); i++)
  {
    joint_vals[traj.joint_names[i]] = traj.points.back().positions[i];
  }

  current_robot_state_->setKinematicState(joint_vals);
}

bool RobotPickPlaceNavigator::getAndSetPlanningScene()
{
  ros::WallTime start_time = ros::WallTime::now();
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

  revertPlanningScene();

  planning_scene_req.planning_scene_diff = planning_scene_diff_;

  ROS_DEBUG_STREAM(NODE_NAME<<": Getting and setting planning scene");

  if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN_STREAM(NODE_NAME<<": Can't get planning scene");
    return false;
  }

  current_planning_scene_ = planning_scene_res.planning_scene;
  current_robot_state_ = cm_.setPlanningScene(current_planning_scene_);

  if(current_robot_state_ == NULL)
  {
    ROS_WARN_STREAM(NODE_NAME<<": Problems setting robot kinematic state");
    return false;
  }

  // Change time stamp to avoid saving sim time.
  current_planning_scene_.robot_state.joint_state.header.stamp = ros::Time(ros::WallTime::now().toSec());
  ROS_INFO_STREAM(NODE_NAME<<": Setting took " << (ros::WallTime::now()-start_time).toSec());
  planning_scene_duration_ += ros::WallTime::now()-start_time;
  return true;
}

bool RobotPickPlaceNavigator::moveArm(const std::string& group_name,const std::vector<double>& joint_positions)
{

  // requesting path plan
  ros::WallTime start_time = ros::WallTime::now();
  arm_navigation_msgs::GetMotionPlan::Request plan_req;
  arm_navigation_msgs::GetMotionPlan::Response plan_res;

  plan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  plan_req.motion_plan_request.group_name = group_name;

  planning_environment::convertKinematicStateToRobotState(*current_robot_state_,
                                                          ros::Time::now(),
                                                          cm_.getWorldFrameId(),
                                                          plan_req.motion_plan_request.start_state);

  std::vector<std::string> joint_names = getJointNames(group_name);
  std::vector<arm_navigation_msgs::JointConstraint>& joint_constraints =  plan_req.motion_plan_request.goal_constraints.joint_constraints;

  joint_constraints.resize(joint_names.size());
  for(unsigned int i = 0; i < joint_names.size(); i++)
  {
    joint_constraints[i].joint_name = joint_names[i];
    joint_constraints[i].position = joint_positions[i];
    joint_constraints[i].tolerance_above = .01;
    joint_constraints[i].tolerance_below = .01;
  }

  if(!planning_service_client_.call(plan_req, plan_res))
  {
    ROS_WARN_STREAM(NODE_NAME<<": Planner service call failed");
    return false;
  }

  if(plan_res.error_code.val != plan_res.error_code.SUCCESS)
  {
    ROS_WARN_STREAM(NODE_NAME<<": Planner failed");
    return false;
  }

  motion_planning_duration_ += ros::WallTime::now()-start_time;
  start_time = ros::WallTime::now();

  last_mpr_id_ = max_mpr_id_;
  max_mpr_id_++;

  // requesting planned trajectory filtering
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;

  filter_req.trajectory = plan_res.trajectory.joint_trajectory;
  filter_req.start_state = plan_req.motion_plan_request.start_state;
  filter_req.group_name = group_name;
  filter_req.goal_constraints = plan_req.motion_plan_request.goal_constraints;
  filter_req.allowed_time = ros::Duration(4.0);

  if(!trajectory_filter_service_client_.call(filter_req, filter_res)) {
    ROS_WARN_STREAM(NODE_NAME<<": Filter service call failed");
    return false;
  }

  if(filter_res.error_code.val != filter_res.error_code.SUCCESS) {
    ROS_WARN_STREAM(NODE_NAME<<": Filter failed");
    return false;
  }
  trajectory_filtering_duration_ += ros::WallTime::now()-start_time;

  // requesting trajectory execution action
  trajectories_succeeded_ = false;
  TrajectoryExecutionRequest ter;
  ter.group_name_ = group_name;
  ter.controller_name_ = arm_controller_handler_->getControllerName();
  ter.trajectory_ = filter_res.trajectory;
  ter.test_for_close_enough_ = false;
  ter.failure_time_factor_ = 1000;
  ter.max_joint_distance_ = .01;

  std::vector<TrajectoryExecutionRequest> ter_reqs;
  ter_reqs.push_back(ter);

  ros::WallTime start_execution = ros::WallTime::now();

  ROS_INFO_STREAM(NODE_NAME<<": Should be trying to move arm");
  trajectory_execution_monitor_.executeTrajectories(ter_reqs,trajectories_finished_function_);

  boost::unique_lock<boost::mutex> lock(execution_mutex_);
  execution_completed_.wait(lock);
  execution_duration_ += (ros::WallTime::now()-start_execution);

  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  if(trajectories_succeeded_)
  {
    error_code.val = error_code.SUCCESS;
    ROS_INFO_STREAM(NODE_NAME<<": Trajectory execution has succeeded");
  }
  else
  {
    error_code.val = error_code.PLANNING_FAILED;
    ROS_INFO_STREAM(NODE_NAME<<": Trajectory execution has failed");
  }

  return trajectories_succeeded_;
}

bool RobotPickPlaceNavigator::validateJointTrajectory(trajectory_msgs::JointTrajectory &jt)
{
	for(unsigned int i = 0; i < jt.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint &p = jt.points[i];
		if(p.positions.size() == 0)
		{
			return false; // should at least have position values
		}

		if(p.velocities.size() == 0)
		{
			p.velocities = std::vector<double>(jt.joint_names.size(),0.0f);
		}

		if(p.accelerations.size() == 0)
		{
			p.accelerations = std::vector<double>(jt.joint_names.size(),0.0f);
		}
	}

	return true;
}

void RobotPickPlaceNavigator::printJointTrajectory(const trajectory_msgs::JointTrajectory &jt)
{

	ROS_INFO_STREAM(NODE_NAME<<": Total joint points: "<<jt.points.size()<<", total joints "<<jt.joint_names.size());
	std::stringstream ss;
	for(unsigned int j = 0; j < jt.points.size(); j++)
	{
		ROS_INFO_STREAM(NODE_NAME<<": joint point time: "<<jt.points[j].time_from_start.toSec());
		for(unsigned int i = 0;i < jt.joint_names.size(); i++)
		{
			ss<<NODE_NAME<<": joint "<<jt.joint_names[i]<<", pos: ";
			if(jt.points[j].positions.size() == 0)
			{
				ss<<"NAN";
			}
			else
			{
				ss<<jt.points[j].positions[i];
			}

			ss<<", vel: ";
			if(jt.points[j].velocities.size() == 0)
			{
				ss<<"NAN";
			}
			else
			{
				ss<<jt.points[j].velocities[i];
			}

			ss<<", acc: ";
			if(jt.points[j].velocities.size() == 0)
			{
				ss<<"NAN";
			}
			else
			{
				ss<<jt.points[j].accelerations[i];
			}

			ROS_INFO_STREAM(ss.str());
			ss.str("");
		}
	}
}

bool RobotPickPlaceNavigator::fastFilterTrajectory(const std::string& group_name,trajectory_msgs::JointTrajectory& jt)
{
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;

  ROS_INFO_STREAM(NODE_NAME<<": Performing fast filter trajectory");
  ROS_INFO_STREAM(NODE_NAME<<": Incoming number of joints: " << jt.joint_names.size());
  ROS_INFO_STREAM(NODE_NAME<<": Incoming number of points: " << jt.points.size());

  std::map<std::string, double> jvals;
  for(unsigned int i = 0; i < jt.joint_names.size(); i++)
  {
    jvals[jt.joint_names[i]] = jt.points[0].positions[i];

    ROS_INFO_STREAM(NODE_NAME<<": Populating joint names: " << jt.joint_names[i]
                    << ", jvals: " << jvals[jt.joint_names[i]]
                    << ", jt.points: " << jt.points[0].positions[i]);
  }

  planning_models::KinematicState state(*current_robot_state_);
  state.setKinematicState(jvals);

  filter_req.trajectory = jt;
  planning_environment::convertKinematicStateToRobotState(state,
                                                          ros::Time::now(),
                                                          cm_.getWorldFrameId(),
                                                          filter_req.start_state);
  filter_req.group_name = group_name;

  if(!trajectory_filter_service_client_.call(filter_req, filter_res))
  {
    ROS_WARN_STREAM(NODE_NAME<<": Fast Filter service call failed");
    ROS_WARN_STREAM(NODE_NAME<<": Return true anyway");
    return true;
  }

  if (filter_res.trajectory.points.size() != 0)
  {
      jt = filter_res.trajectory;
  }
  else
  {
      ROS_WARN_STREAM(NODE_NAME<<": Filter returned an empty trajectory (ignoring for now)");
  }

  ROS_INFO_STREAM(NODE_NAME<<": Filtered number of joints: " << jt.joint_names.size());
  ROS_INFO_STREAM(NODE_NAME<<": Filtered number of points: " << jt.points.size());
  return true;
}

bool RobotPickPlaceNavigator::moveArmToSide()
{

    getAndSetPlanningScene();

    //name: ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
    //position: [-0.46665400266647339, 0.1069866344332695, 2.1171059608459473, -1.4666222333908081, -0.17949093878269196, -1.6554385423660278, -1.7431133985519409]

    //-3.0410828590393066, 0.10696959495544434, 2.117098093032837, -1.466621994972229, -0.17949093878269196, -1.6554234027862549, -1.7430833578109741

    //position: [0.16656530392591254, 0.46065822721369176, 2.4644586717834467, 0.49449136755439443, -0.2900361153401066, 1.4113548618662812, 2.3286899342716625]


//    std::vector<double> joint_angles;
//    joint_angles.push_back(-1.0410828590393066);
//    joint_angles.push_back(0.46065822721369176);
//    joint_angles.push_back(2.4644586717834467);
//    joint_angles.push_back(0.49449136755439443);
//    joint_angles.push_back(-0.2900361153401066);
//    joint_angles.push_back(1.4113548618662812);
//    joint_angles.push_back(2.3286899342716625);
    _JointConfigurations.fetchParameters(JOINT_CONFIGURATIONS_NAMESPACE);
    return moveArm(arm_group_name_,_JointConfigurations.SideAngles);
}

void RobotPickPlaceNavigator::addDetectedTableToPlanningSceneDiff(const tabletop_object_detector::Table &table)
{
  arm_navigation_msgs::CollisionObject table_object;
  table_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  table_object.shapes.resize(1);
  table_object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
  table_object.shapes[0].dimensions.resize(3);
  table_object.shapes[0].dimensions[0] = fabs(table.x_max-table.x_min);
  table_object.shapes[0].dimensions[1] = fabs(table.y_max-table.y_min);
  table_object.shapes[0].dimensions[2] = 0.01;
  table_object.id = "table";

  // finding table transform in world coordinates
  geometry_msgs::PoseStamped table_world;
  cm_.convertPoseGivenWorldTransform(*current_robot_state_,
                                     cm_.getWorldFrameId(),
                                     table.pose.header,
                                     table.pose.pose,
                                     table_world);

  table_object.header = table_world.header;

  //set the origin of the table object in the middle of the table
  tf::Transform table_trans;
  tf::poseMsgToTF(table_world.pose, table_trans);
  tf::Transform table_translation;
  table_translation.setIdentity();
  table_translation.setOrigin( tf::Vector3( (table.x_min + table.x_max)/2.0, (table.y_min + table.y_max)/2.0, 0.0) );
  table_trans = table_trans * table_translation;
  table_object.poses.resize(1);
  tf::poseTFToMsg(table_trans, table_object.poses[0]);


  ROS_INFO_STREAM(NODE_NAME<<": Table location is " << table_object.poses[0].position.x << " "
                  << table_object.poses[0].position.y << " "
                  << table_object.poses[0].position.z);

  // saving results from table object
  planning_scene_diff_.collision_objects.push_back(table_object);
  table_ = table_object;
}

void RobotPickPlaceNavigator::addDetectedObjectToPlanningSceneDiff(arm_navigation_msgs::CollisionObject &obj)
{
  obj.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  planning_scene_diff_.collision_objects.push_back(obj);
}

void RobotPickPlaceNavigator::addDetectedObjectToPlanningSceneDiff(const household_objects_database_msgs::DatabaseModelPoseList& model)
{
  arm_navigation_msgs::CollisionObject obj;
  obj.id = makeCollisionObjectNameFromModelId(model.model_list[0].model_id);

  // finding pose of object in world coordinates
  geometry_msgs::PoseStamped obj_world;
  cm_.convertPoseGivenWorldTransform(*current_robot_state_,
                                     cm_.getWorldFrameId(),
                                     model.model_list[0].pose.header,
                                     model.model_list[0].pose.pose,
                                     obj_world);

  obj.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  getMeshFromDatabasePose(model.model_list[0],
                          obj,
                          obj_world);

  ROS_INFO_STREAM(NODE_NAME<<": Pushing back " << obj.id << " frame " << obj.header.frame_id);
  planning_scene_diff_.collision_objects.push_back(obj);
}

bool RobotPickPlaceNavigator::performSegmentation()
{
	//  ===================================== saving current time stamp =====================================
	ros::WallTime start  = ros::WallTime::now();

	//  ===================================== clearing results from last call =====================================
	planning_scene_diff_.collision_objects.clear();
	recognized_obj_pose_map_.clear();

	//  ===================================== calling service =====================================
	tabletop_object_detector::TabletopSegmentation segmentation_srv;
	bool success = seg_srv_.call(segmentation_srv);

	// printing timing
	ros::WallTime after_seg = ros::WallTime::now();
	ROS_INFO_STREAM(NODE_NAME<<": Seg took " << (after_seg-start).toSec());

	// ===================================== checking results ========================================
	if (!success)
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Call to segmentation service failed");
		return false;
	}

	if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Segmentation service returned error "<< segmentation_srv.response.result);
		return false;
	}

	if(segmentation_srv.response.clusters.size() == 0)
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Tabletop segmentation returned 0 clusters, exiting");
		return false;
	}

	//  ===================================== storing results =====================================
	segmentation_results_ = segmentation_srv.response;
	segmented_clusters_ = segmentation_srv.response.clusters;

	//  ===================================== updating local planning scene =====================================
	addDetectedTableToPlanningSceneDiff(segmentation_srv.response.table);

	// ===================================== printing completion info message =====================================
	ROS_INFO_STREAM(NODE_NAME<<": Segmentation service succeeded. Detected "<<(int)segmentation_srv.response.clusters.size()<<" clusters");

	return true;
}

bool RobotPickPlaceNavigator::performSphereSegmentation()
{
	//  ===================================== preparing result objects =====================================
	  arm_navigation_msgs::CollisionObject obj;
	  int bestClusterIndex = -1;
	  sensor_msgs::PointCloud sphereCluster;

	  //  ===================================== calling segmentation =====================================
	  _SphereSeg.fetchParameters(SEGMENTATION_NAMESPACE);
	  bool success = _SphereSeg.segment(segmented_clusters_,obj,bestClusterIndex);

	  // ===================================== checking results ========================================
	  if(!success)
	  {
		  ROS_ERROR_STREAM(NODE_NAME<<": Sphere segmentation did not succeed");
		  return false;
	  }

	  //  ===================================== storing results =====================================
	  // assigning id first
	  obj.id = makeCollisionObjectNameFromModelId(0);

	  // retrieving segmented sphere cluster
	  _SphereSeg.getSphereCluster(sphereCluster);

	  // storing best cluster
	  segmented_clusters_.clear();
	  segmented_clusters_.push_back(sphereCluster);

	  // storing pose
	  geometry_msgs::PoseStamped pose;
	  pose.header.frame_id = obj.header.frame_id;
	  pose.pose = obj.poses[0];
	  recognized_obj_pose_map_[std::string(obj.id)] = pose;

	  // storing recognized object as model for planning
	  household_objects_database_msgs::DatabaseModelPoseList models;
	  household_objects_database_msgs::DatabaseModelPose model;
	  model.model_id = 0;
	  model.pose = pose;
	  model.confidence = 1.0f;
	  model.detector_name = "sphere_segmentation";
	  models.model_list.push_back(model);
	  recognized_models_.clear();
	  recognized_models_.push_back(models);

	  //  ===================================== updating local planning scene =====================================
	  addDetectedObjectToPlanningSceneDiff(obj);

	  //  ===================================== updating markers =====================================
	  // update markers
	  visualization_msgs::Marker &segMarker = MarkerMap[MARKER_SEGMENTED_OBJ];
	  visualization_msgs::Marker &attachedMarker = MarkerMap[MARKER_ATTACHED_OBJ];

	  // segmented object
	  collisionObjToMarker(obj,segMarker);
	  segMarker.action = visualization_msgs::Marker::ADD;

	  // attached object, hide for now until gripper is close
	  collisionObjToMarker(obj,attachedMarker);
	  attachedMarker.action = visualization_msgs::Marker::DELETE;
	  attachedMarker.header.frame_id = gripper_link_name_;

	  // ===================================== printing completion info message =====================================
	  arm_navigation_msgs::Shape &shape = obj.shapes[0];
	  ROS_INFO_STREAM("\n"<<NODE_NAME<<": Sphere Segmentation completed:\n");
	  ROS_INFO_STREAM("\tFrame id: "<<obj.header.frame_id<<"\n");
	  ROS_INFO_STREAM("\tRadius: "<<shape.dimensions[0]<<"\n");
	  ROS_INFO_STREAM("\tx: "<<pose.pose.position.x<<", y: "<<pose.pose.position.y
			  <<", z: "<<pose.pose.position.z<<"\n");

	return true;
}

bool RobotPickPlaceNavigator::performRecognition()
{
	//  ===================================== saving current time stamp =====================================
	ros::WallTime start = ros::WallTime::now();

	//  ===================================== clearing results from last call =====================================
	planning_scene_diff_.collision_objects.clear();
	recognized_obj_pose_map_.clear();

	//  ===================================== calling service =====================================
	tabletop_object_detector::TabletopObjectRecognition recognition_srv;
	recognition_srv.request.table = segmentation_results_.table;
	recognition_srv.request.clusters = segmented_clusters_;
	recognition_srv.request.num_models = 1;
	recognition_srv.request.perform_fit_merge = false;

	bool success = rec_srv_.call(recognition_srv);

	// ===================================== checking results ========================================
	if (!success)
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Call to recognition service " <<recognition_service_.c_str()<<" failed.");
		return false;
	}

    if(recognition_srv.response.models.size() == 0)
    {
    	ROS_ERROR_STREAM(NODE_NAME<<": Recognition found no objects");
    	return false;
    }

	//  ===================================== storing results =====================================
    recognized_models_ = recognition_srv.response.models;


    //  ===================================== calling service =====================================
    household_objects_database_msgs::GetModelDescription::Request des_req;
    household_objects_database_msgs::GetModelDescription::Response des_res;
    des_req.model_id = recognized_models_[0].model_list[0].model_id;

    success = object_database_model_description_client_.call(des_req, des_res);

	// ===================================== checking results ========================================
	if(!success)
	{
		ROS_WARN_STREAM(NODE_NAME<<": Call to objects database for getModelDescription failed");
		return false;
	}

	if(des_res.return_code.code != des_res.return_code.SUCCESS)
	{
		ROS_WARN_STREAM(NODE_NAME<<":Object database gave non-success code "
				<< des_res.return_code.code
				<< " for model id "
				<< des_req.model_id);
		return false;
	}

	//  ===================================== storing results =====================================
	recognized_model_description_ = des_res;

	//  ===================================== updating local planning scene =====================================
    addDetectedObjectToPlanningSceneDiff(recognition_srv.response.models[0]);

	// ===================================== printing completion info message =====================================
    ROS_INFO_STREAM(NODE_NAME<<": Recognition took " << (ros::WallTime::now()-start));
    ROS_INFO_STREAM(NODE_NAME<<": Got " << recognition_srv.response.models.size() << " models");
    recognized_models_ = recognition_srv.response.models;
	std::stringstream stdout;
	stdout<<"\nRetrieved models:";
    BOOST_FOREACH(household_objects_database_msgs::DatabaseModelPose model,recognition_srv.response.models[0].model_list)
    {
    	stdout<<"\n\tModel id: "<<model.model_id;
    	stdout<<"\n\tPose frame id: "<<model.pose.header.frame_id;
    	stdout<<"\n\tdetector name: "<<model.detector_name;
    	stdout<<"\n";
    }
    ROS_INFO_STREAM(stdout.str());
    ROS_INFO_STREAM(NODE_NAME<<" model database service returned description with name: "<<des_res.name);

	return true;
}

bool RobotPickPlaceNavigator::getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
                             arm_navigation_msgs::CollisionObject& obj,
                             const geometry_msgs::PoseStamped& pose)
{
  household_objects_database_msgs::GetModelMesh::Request req;
  household_objects_database_msgs::GetModelMesh::Response res;

  req.model_id = model_pose.model_id;
  if(!object_database_model_mesh_client_.call(req, res))
  {
    ROS_WARN_STREAM("Call to objects database for getMesh failed");
    return false;
  }
  if(res.return_code.code != res.return_code.SUCCESS) {
    ROS_WARN_STREAM("Object database gave non-success code " << res.return_code.code << " for model id " << req.model_id);
    return false;
  }

  obj.header = pose.header;
  obj.poses.resize(1);
  obj.shapes.resize(1);

  recognized_obj_pose_map_[obj.id] = pose;

  bool use_cylinder = true;

  if(use_cylinder) {
    tf::Transform pose_tf;
    tf::poseMsgToTF(pose.pose, pose_tf);

    shapes::Shape* shape = planning_environment::constructObject(res.mesh);
    bodies::Body* body = bodies::createBodyFromShape(shape);
    body->setPose(pose_tf);

    bodies::BoundingCylinder cyl;
    body->computeBoundingCylinder(cyl);

    obj.shapes[0].type = arm_navigation_msgs::Shape::CYLINDER;
    obj.shapes[0].dimensions.resize(2);
    obj.shapes[0].dimensions[0] = cyl.radius;
    obj.shapes[0].dimensions[1] = cyl.length;
    tf::poseTFToMsg(cyl.pose, obj.poses[0]);
  } else {
    obj.shapes.resize(1);
    obj.shapes[0].type = arm_navigation_msgs::Shape::MESH;
    obj.shapes[0] = res.mesh;
    obj.poses[0] = pose.pose;
  }
  return true;
}

bool RobotPickPlaceNavigator::performGraspPlanning()
{
	//  ===================================== saving current time stamp =====================================
	ros::WallTime start_time = ros::WallTime::now();

	//  ===================================== clearing results from last call =====================================
	grasp_pickup_goal_.target.potential_models.clear();
	grasp_candidates_.clear();

	//  ===================================== calling service =====================================
	// checking available recognized models
	if((recognized_models_.size() == 0) || (recognized_models_[0].model_list.size() == 0))
	{
	  return false;
	}

	// populating grasp plan request/response
	household_objects_database_msgs::DatabaseModelPose modelPose = recognized_models_[0].model_list[0];
	object_manipulation_msgs::GraspPlanning::Request request;
	object_manipulation_msgs::GraspPlanning::Response response;
	std::string modelId = makeCollisionObjectNameFromModelId(modelPose.model_id);

	//modelPose.pose = geometry_msgs::PoseStamped();
	//modelPose.pose.pose.orientation.w = 1.0;
	modelPose.pose = recognized_obj_pose_map_[modelId];
	modelPose.pose.header.frame_id = recognized_model_description_.name; // should be updated during recognition stage
	modelPose.pose.header.stamp = ros::Time::now();
	request.arm_name = arm_group_name_;
	request.target.potential_models.push_back(modelPose);
	request.target.reference_frame_id = segmented_clusters_[0].header.frame_id;
	request.target.cluster = segmented_clusters_[0];

	bool success = grasp_planning_client.call(request, response);

	// ===================================== checking results ========================================
	if(!success)
	{
		ROS_WARN_STREAM(NODE_NAME<<": grasp planning call unsuccessful, exiting");
		return false;
	}

	if(response.error_code.value != response.error_code.SUCCESS)
	{
		ROS_WARN_STREAM(NODE_NAME<<": grasp planning call returned error code, exiting " << response.error_code.value);
		return false;
	}

	if(response.grasps.size() == 0)
	{
		ROS_WARN_STREAM(NODE_NAME<<": No grasps returned in response");
		return false;
	}

	//TODO - actually deal with the different cases here, especially for the cluster planner
	if(request.target.reference_frame_id != recognized_model_description_.name ||
		  request.target.reference_frame_id != modelPose.pose.header.frame_id)
	{
	  ROS_WARN_STREAM("Cluster does not match recognition");
	}

	//  ===================================== storing results =====================================
	/* Storing grasp candidates:
	 * 	Grasp poses are positions of the wrist link in terms of the world,
	 * 	we need to get them in terms of the object
	 */
	grasp_candidates_.assign(response.grasps.begin(),response.grasps.end());

	// instantiating needed transforms and poses
	tf::Transform object_in_world_tf;
	tf::StampedTransform wristInGripperTcp = tf::StampedTransform();
	tf::Transform object_in_world_inverse_tf;

	// populating transforms
	tf::poseMsgToTF(modelPose.pose.pose, object_in_world_tf);
	object_in_world_inverse_tf = object_in_world_tf.inverse();
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wristInGripperTcp);

	// applying transformation to grasp pose so that the arm wrist relative to the object is obtained
	for(unsigned int i = 0; i < grasp_candidates_.size(); i++)
	{
		tf::Transform grasp_in_world_tf;
		tf::poseMsgToTF(grasp_candidates_[i].grasp_pose, grasp_in_world_tf);
		tf::poseTFToMsg(object_in_world_inverse_tf*(grasp_in_world_tf*wristInGripperTcp),
			  grasp_candidates_[i].grasp_pose);
	}

	// storing grasp pickup goal to be used later during pick move sequence execution
	grasp_pickup_goal_.arm_name = arm_group_name_;
	grasp_pickup_goal_.collision_object_name = modelId;
	grasp_pickup_goal_.lift.direction.header.frame_id = cm_.getWorldFrameId();
	grasp_pickup_goal_.lift.direction.vector.z = 1.0;
	grasp_pickup_goal_.lift.desired_distance = .1;
	grasp_pickup_goal_.target.reference_frame_id = modelId;
	grasp_pickup_goal_.target.cluster = segmented_clusters_[0];
	grasp_pickup_goal_.allow_gripper_support_collision = true;
	grasp_pickup_goal_.collision_support_surface_name = "table";
	grasp_pickup_goal_.target.potential_models.push_back(modelPose);

	//  ===================================== updating local planning scene =====================================
	// updating gripper (not sure if this is necessary)
	tf::Transform first_grasp_in_world_tf;
	tf::poseMsgToTF(response.grasps[0].grasp_pose, first_grasp_in_world_tf);
	planning_models::KinematicState state(*current_robot_state_);
	state.updateKinematicStateWithLinkAt(gripper_link_name_, first_grasp_in_world_tf);

	//  ===================================== publishing results =====================================
	// publishing marker of gripper at pre-grasp
	visualization_msgs::MarkerArray arr;
	std_msgs::ColorRGBA col_pregrasp;
	col_pregrasp.r = 0.0;
	col_pregrasp.g = 1.0;
	col_pregrasp.b = 1.0;
	col_pregrasp.a = 1.0;
	std::vector<std::string> links = cm_.getKinematicModel()->getModelGroup(gripper_group_name_)->getGroupLinkNames();
	cm_.getRobotMarkersGivenState(state, arr, col_pregrasp,"first_grasp", ros::Duration(0.0), &links);
	vis_marker_array_publisher_.publish(arr);

	// ===================================== printing completion info message =====================================
	ROS_INFO_STREAM(NODE_NAME<<": Grasp is " << response.grasps[0].grasp_pose.position.x << " "
			  << response.grasps[0].grasp_pose.position.y << " "
			  << response.grasps[0].grasp_pose.position.z);

	ROS_INFO_STREAM(NODE_NAME<<": Cloud header " << segmented_clusters_[0].header.frame_id);
	ROS_INFO_STREAM(NODE_NAME<<": Recognition pose frame " << modelPose.pose.header.frame_id);

	return true;
}

void RobotPickPlaceNavigator::createPickMoveSequence(
		const object_manipulation_msgs::PickupGoal &pickupGoal,
		const std::vector<object_manipulation_msgs::Grasp> &grasps,
		std::vector<object_manipulator::GraspExecutionInfo> &graspSequence)
{
    ROS_INFO_STREAM(NODE_NAME<<": Evaluating grasps with grasp tester");

	planning_models::KinematicState kinematicState(*current_robot_state_);
	grasp_tester_->setPlanningSceneState(&kinematicState);
	grasp_tester_->testGrasps(pickupGoal,grasps,graspSequence,true);

    ROS_INFO_STREAM(NODE_NAME<<": Returned "<< graspSequence.size() <<" grasps candidates ");
}

void RobotPickPlaceNavigator::createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &placeGoal,
				const std::vector<geometry_msgs::PoseStamped> &placePoses,
				std::vector<object_manipulator::PlaceExecutionInfo> &placeSequence)
{
	// find transform of wrist relative to the gripper's tcp
	tf::StampedTransform gripperTcpToWrist = tf::StampedTransform();
	planning_models::KinematicState state(*current_robot_state_);
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),gripperTcpToWrist);
	place_tester_->setTcpToWristTransform(gripperTcpToWrist);
	place_tester_->setPlanningSceneState(&state);
	place_tester_->testPlaces(placeGoal, placePoses, placeSequence, true);
}

bool RobotPickPlaceNavigator::moveArmThroughPickSequence()
{
	// pushing local changes to planning scene
	getAndSetPlanningScene();

	// grasp planning
	bool success = performGraspPlanning();
	if(!success)
	{
		ROS_INFO_STREAM(NODE_NAME<<": Pick Move attempt aborted");
		return false;
	}

	// creating pick move sequence
	std::vector<object_manipulator::GraspExecutionInfo> graspSequence;
	createPickMoveSequence(grasp_pickup_goal_,grasp_candidates_,graspSequence);

	// try each successful grasp
	success = false;
	int counter = 0; // index to grasp array
	BOOST_FOREACH(object_manipulator::GraspExecutionInfo graspMoves,graspSequence)
	{
		bool proceed = (graspMoves.result_.result_code == object_manipulation_msgs::GraspResult::SUCCESS);
		if(proceed)
		{
			ROS_INFO_STREAM(NODE_NAME<<": Attempting Pick grasp sequence");
			success = attemptGraspSequence(arm_group_name_,graspMoves);
			if(!success)
			{
				ROS_INFO_STREAM(NODE_NAME<<": Grasp pick move failed, aborting");
				return false;
			}
			else
			{
			  ROS_INFO_STREAM(NODE_NAME<<": Grasp pick move succeeded");
			}

			// storing current grasp data
			object_manipulation_msgs::Grasp tempGrasp;
			tf::StampedTransform gripperTcpToWrist = tf::StampedTransform();// wrist pose relative to gripper
			tf::Transform wristInObjPose;
			_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),gripperTcpToWrist);

			tf::poseMsgToTF(grasp_candidates_[counter].grasp_pose,wristInObjPose);
			tf::poseTFToMsg(wristInObjPose*(gripperTcpToWrist.inverse()),tempGrasp.grasp_pose);
			current_grasp_map_[arm_group_name_] = tempGrasp;
			current_grasped_object_name_[arm_group_name_] = grasp_pickup_goal_.collision_object_name;

			// updating attached object marker pose
			if(MarkerMap.find(MARKER_ATTACHED_OBJ) != MarkerMap.end())
			{
			  visualization_msgs::Marker &m = MarkerMap[MARKER_ATTACHED_OBJ];
			  tf::poseTFToMsg(wristInObjPose.inverse(),m.pose);
			  m.header.frame_id = gripper_link_name_;
			}

			break;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME<<": Grasp pick move unreachable, skipping to next.");
		}

		counter++;
	}
	return success;
}

bool RobotPickPlaceNavigator::moveArmThroughPlaceSequence()
{
	// resetting scene
	getAndSetPlanningScene();

	// starting timer
	ros::WallTime start_time = ros::WallTime::now();

	// checking grasp
	if(!object_in_hand_map_[arm_group_name_])
	{
		return false;
	}

	// populate grasp place goal
	grasp_place_goal_.arm_name = arm_group_name_;
	grasp_place_goal_.grasp = current_grasp_map_[arm_group_name_];
	grasp_place_goal_.desired_retreat_distance = .1;
	grasp_place_goal_.min_retreat_distance = .1;
	grasp_place_goal_.approach.desired_distance = .1;
	grasp_place_goal_.approach.min_distance = .1;
	grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
	grasp_place_goal_.approach.direction.vector.x = 0.0;
	grasp_place_goal_.approach.direction.vector.y = 0.0;
	grasp_place_goal_.approach.direction.vector.z = -1.0;
	grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];
	grasp_place_goal_.allow_gripper_support_collision = true;
	grasp_place_goal_.collision_support_surface_name = "table";
	grasp_place_goal_.place_padding = .02;

	// creating candidate grasp place poses
	std::vector<geometry_msgs::PoseStamped> placePoses;
	_GoalParameters.fetchParameters(GOAL_NAMESPACE);
	_GoalParameters.generateNextLocationCandidates(placePoses);

	// creating place move sequence
	std::vector<object_manipulator::PlaceExecutionInfo> placeSequence;
	createPlaceMoveSequence(grasp_place_goal_,placePoses,placeSequence);

	// try each place sequence candidate
	bool success = false;
	BOOST_FOREACH(object_manipulator::PlaceExecutionInfo placeMove,placeSequence)
	{
		bool proceed = placeMove.result_.result_code == object_manipulation_msgs::PlaceLocationResult::SUCCESS;

		if(proceed)
		{
			success = attemptPlaceSequence(arm_group_name_,placeMove);
			if(success)
			{
				ROS_INFO_STREAM(NODE_NAME<<": Grasp place move succeeded");
			}
			else
			{
				ROS_ERROR_STREAM(NODE_NAME<<"Grasp place move failed, aborting");
				return false;
			}

			break;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME<<": Grasp place move unreachable, skipping to next.");
		}
	}

	return success;
}

void RobotPickPlaceNavigator::createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	// getting parameters
	_GoalParameters.fetchParameters(GOAL_NAMESPACE);

	// creating place locations array
	int numCandidatePoses = _GoalParameters.NumGoalCandidates;
	const tf::Vector3 &axisRotation = _GoalParameters.Axis;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = _GoalParameters.GoalTransform.frame_id_;

	// rotate about z axis and apply to original goal pose in order to create candidates;
	for(int i = 0; i < numCandidatePoses; i++)
	{
		double ratio = ((double)i)/((double)numCandidatePoses);
		double angle = 2*M_PI*ratio;
		tf::Quaternion q = tf::Quaternion(axisRotation,angle);
		tf::Vector3 p = tf::Vector3(0,0,0);
		tf::Transform candidateTransform = _GoalParameters.GoalTransform*tf::Transform(q,p);
		tf::poseTFToMsg(candidateTransform,pose.pose);
		placePoses.push_back(pose);

		ROS_INFO_STREAM(NODE_NAME<<" :Candidate Goal Position.Pos = ["<<pose.pose.position.x<<", "
				<<pose.pose.position.y<<", "<<pose.pose.position.z<<"]");
		ROS_INFO_STREAM(NODE_NAME<<" :Candidate Goal Position.Frame_Id = "<<pose.header.frame_id);
	}
}

void RobotPickPlaceNavigator::attachCollisionObjectCallback(const std::string& group_name) {
  ROS_INFO_STREAM(NODE_NAME<<": In attach callback");
  attachCollisionObject(group_name,
                        current_grasped_object_name_[group_name],
                        current_grasp_map_[group_name]);
  object_in_hand_map_[group_name] = true;
}

void RobotPickPlaceNavigator::detachCollisionObjectCallback(const std::string& group_name) {
  ROS_INFO_STREAM("In detach callback");
  detachCollisionObject(group_name,
                        current_place_location_,
                        current_grasp_map_[group_name]);
  object_in_hand_map_[group_name] = false;
  current_grasped_object_name_.erase(group_name);
  current_grasp_map_.erase(group_name);
}

bool RobotPickPlaceNavigator::attemptGraspSequence(const std::string& group_name,
                          const object_manipulator::GraspExecutionInfo& gei) {

  std::vector<std::string> segment_names;

  //first open the gripper
  std::vector<TrajectoryExecutionRequest> ter_reqs;
  TrajectoryExecutionRequest gripper_ter;
  gripper_ter.group_name_ = gripper_group_name_;
  gripper_ter.controller_name_ = grasp_action_service_;
  gripper_ter.trajectory_ = getGripperTrajectory(group_name, true);
  gripper_ter.failure_ok_ = true;
  gripper_ter.test_for_close_enough_ = false;
  ter_reqs.push_back(gripper_ter);

  ros::WallTime start_execution = ros::WallTime::now();
  ROS_INFO_STREAM(NODE_NAME << ": Open gripper trajectory in progress");
  trajectory_execution_monitor_.executeTrajectories(ter_reqs,trajectories_finished_function_);
  {
    boost::unique_lock<boost::mutex> lock(execution_mutex_);
    execution_completed_.wait(lock);
  }

  execution_duration_ += (ros::WallTime::now()-start_execution);
  ROS_INFO_STREAM(NODE_NAME << ": Open gripper trajectory completed");
  ter_reqs.clear();

  // moving arm from current configuration to pre-grasp configuration
  updateCurrentJointStateToLastTrajectoryPoint(last_trajectory_execution_data_vector_.back().recorded_trajectory_);
  ROS_INFO_STREAM(NODE_NAME << ": arm approach trajectory in progress");
  if(!moveArm(group_name, gei.approach_trajectory_.points[0].positions))
  {
    return false;
  }
  ROS_INFO_STREAM(NODE_NAME << ": arm approach trajectory completed");
  trajectories_succeeded_ = false;

  // request gripper pre-grasp command
  object_manipulation_msgs::GraspHandPostureExecutionGoal graspGoal;
  graspGoal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP;

  ROS_INFO_STREAM(NODE_NAME << ": Requesting pre-grasp");
  grasp_exec_action_client_->sendGoal(graspGoal);
  if(!grasp_exec_action_client_->waitForResult(ros::Duration(2.0f)))
  {
	  ROS_ERROR_STREAM(NODE_NAME << ": Pre-grasp request timeout, exiting");
	  return false;
  }

  if(grasp_exec_action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
	  ROS_ERROR_STREAM(NODE_NAME << ": Pre-grasp request unsuccessful, exiting");
	  return false;
  }
  else
  {
	  ROS_INFO_STREAM(NODE_NAME << ": Pre-grasp completed");
  }


  //now do approach
  TrajectoryExecutionRequest arm_ter;
  arm_ter.group_name_ = group_name;
  arm_ter.controller_name_ = arm_controller_handler_->getControllerName();
  arm_ter.trajectory_ = gei.approach_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  fastFilterTrajectory(arm_ter.group_name_, arm_ter.trajectory_);
  arm_ter.test_for_close_enough_ = false;
  arm_ter.failure_ok_ = false;
  arm_ter.max_joint_distance_ = .01;
  arm_ter.failure_time_factor_ = 1000;
  arm_ter.callback_function_ = boost::bind(&RobotPickPlaceNavigator::attachCollisionObjectCallback, this, _1);
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("approach");

  //now close the gripper
  gripper_ter.trajectory_ = getGripperTrajectory(group_name, false);
  validateJointTrajectory(gripper_ter.trajectory_);
  ter_reqs.push_back(gripper_ter);
  segment_names.push_back("close");

  // updating attached  marker operation
  if(MarkerMap.find(MARKER_ATTACHED_OBJ) != MarkerMap.end())
  {
	  visualization_msgs::Marker &m = MarkerMap[MARKER_ATTACHED_OBJ];
	  m.action = visualization_msgs::Marker::ADD;
  }

  //and do the lift
  arm_ter.trajectory_ = gei.lift_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  fastFilterTrajectory(group_name, arm_ter.trajectory_);
  arm_ter.callback_function_ = NULL;
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("lift");

  /*
   * executing all trajectories
   * currently this is the only way to execute multiple trajectories in sequence, since the executeTrajectories
   * method can only handle a single trajectory at a time.
   */
  start_execution = ros::WallTime::now();
  for(unsigned int i = 0;i < ter_reqs.size(); i++)
  {
	  std::vector<TrajectoryExecutionRequest> tempRequest;
	  tempRequest.push_back(ter_reqs[i]);
	  ROS_INFO_STREAM(NODE_NAME << ": gripper "<<segment_names[i] <<" trajectory in progress");
	  trajectory_execution_monitor_.executeTrajectories(tempRequest,
														trajectories_finished_function_);
	  {
		  boost::unique_lock<boost::mutex> lock(execution_mutex_);
		  execution_completed_.wait(lock);
	  }
	  ROS_INFO_STREAM(NODE_NAME << ": gripper "<<segment_names[i] <<" trajectory completed");
  }

  execution_duration_ += (ros::WallTime::now()-start_execution);

  return trajectories_succeeded_;
}

bool RobotPickPlaceNavigator::attemptPlaceSequence(const std::string& group_name,
                          const object_manipulator::PlaceExecutionInfo& pei) {
  std::vector<TrajectoryExecutionRequest> ter_reqs;

  if(!moveArm(group_name,
              pei.descend_trajectory_.points[0].positions)) {
    return false;
  }

  trajectories_succeeded_ = false;
  std::vector<std::string> segment_names;

  //now do descend
  TrajectoryExecutionRequest arm_ter;
  arm_ter.group_name_ = group_name;
  arm_ter.controller_name_ = arm_controller_handler_->getControllerName();
  arm_ter.trajectory_ = pei.descend_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  fastFilterTrajectory(group_name, arm_ter.trajectory_);
  arm_ter.test_for_close_enough_ = false;
  arm_ter.max_joint_distance_ = .05;
  arm_ter.failure_time_factor_ = 1000;
  arm_ter.callback_function_ = boost::bind(&RobotPickPlaceNavigator::detachCollisionObjectCallback, this, _1);
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("descend");

  //open gripper
  TrajectoryExecutionRequest gripper_ter;
  gripper_ter.group_name_ = gripper_group_name_;
  gripper_ter.controller_name_ = grasp_action_service_;
  gripper_ter.trajectory_ = getGripperTrajectory(group_name, true);
  validateJointTrajectory(gripper_ter.trajectory_);
  gripper_ter.failure_ok_ = true;
  gripper_ter.test_for_close_enough_ = false;
  ter_reqs.push_back(gripper_ter);
  segment_names.push_back("open_gripper");

  // updating attached objt marker operation
  if(MarkerMap.find(MARKER_ATTACHED_OBJ) != MarkerMap.end())
  {
	  visualization_msgs::Marker &m = MarkerMap[MARKER_ATTACHED_OBJ];
	  m.action = visualization_msgs::Marker::DELETE;
  }

  //do the retreat
  arm_ter.trajectory_ = pei.retreat_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  fastFilterTrajectory(group_name, arm_ter.trajectory_);
  arm_ter.callback_function_ = 0;
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("retreat");

  /*
   * executing all trajectories
   * currently this is the only way to execute multiple trajectories in sequence, since the executeTrajectories
   * method can only handle a single trajectory at a time.
   */
  ros::WallTime start_execution = ros::WallTime::now();
  for(unsigned int i = 0;i < ter_reqs.size(); i++)
  {
	  std::vector<TrajectoryExecutionRequest> tempRequest;
	  tempRequest.push_back(ter_reqs[i]);
	  ROS_INFO_STREAM(NODE_NAME << ": gripper "<<segment_names[i] <<" trajectory in progress");
	  trajectory_execution_monitor_.executeTrajectories(tempRequest,
														trajectories_finished_function_);
	  {
		  boost::unique_lock<boost::mutex> lock(execution_mutex_);
		  execution_completed_.wait(lock);
	  }
	  ROS_INFO_STREAM(NODE_NAME << ": gripper "<<segment_names[i] <<" trajectory completed");
  }

  execution_duration_ += (ros::WallTime::now()-start_execution);
  return trajectories_succeeded_;
}

trajectory_msgs::JointTrajectory RobotPickPlaceNavigator::getGripperTrajectory(const std::string& arm_name,bool open)
{
  trajectory_msgs::JointTrajectory gt;
  if(arm_name == "right_arm") {
    gt.joint_names.push_back("r_gripper_l_finger_joint");
    gt.joint_names.push_back("r_gripper_r_finger_joint");
    gt.joint_names.push_back("r_gripper_r_finger_tip_joint");
    gt.joint_names.push_back("r_gripper_l_finger_tip_joint");
  } else {
    gt.joint_names.push_back("l_gripper_l_finger_joint");
    gt.joint_names.push_back("l_gripper_r_finger_joint");
    gt.joint_names.push_back("l_gripper_r_finger_tip_joint");
    gt.joint_names.push_back("l_gripper_l_finger_tip_joint");
  }
  gt.points.resize(1);
  if(open) {
    gt.points[0].positions.resize(4, .1);
  } else {
    gt.points[0].positions.resize(4, 0.0);
  }
  return gt;
}

bool RobotPickPlaceNavigator::attachCollisionObject(const std::string& group_name,
                           const std::string& collision_object_name,
                           const object_manipulation_msgs::Grasp& grasp) {

  // removing object from planning scene first if it is found
  arm_navigation_msgs::AttachedCollisionObject att;
  bool found = false;
  for(std::vector<arm_navigation_msgs::CollisionObject>::iterator it = planning_scene_diff_.collision_objects.begin();
      it != planning_scene_diff_.collision_objects.end();
      it++)
  {

    if((*it).id == collision_object_name)
    {
      found = true;
      att.object = (*it);
      planning_scene_diff_.collision_objects.erase(it);
      break;
    }
  }


  if(!found)
  {
    ROS_ERROR_STREAM("No object with id " << collision_object_name);
    return false;
  }

  att.link_name = gripper_link_name_;
  att.touch_links.push_back(gripper_group_name_);

  planning_models::KinematicState state(*current_robot_state_);
  tf::Transform t;
  tf::poseMsgToTF(grasp.grasp_pose, t);

  std::map<std::string, double> grasp_vals;
  for(unsigned int i = 0; i < grasp.grasp_posture.name.size(); i ++)
  {
     grasp_vals[grasp.grasp_posture.name[i]] = grasp.grasp_posture.position[i];
  }
  state.setKinematicState(grasp_vals);

  tf::Transform obj_pose;
  tf::poseMsgToTF(recognized_obj_pose_map_[collision_object_name].pose, obj_pose);// store object pose in world coordinates

  //assumes that the grasp is in the object frame
  tf::Transform grasp_pose = obj_pose*t;

  // //need to do this second, otherwise it gets wiped out
  state.updateKinematicStateWithLinkAt(gripper_link_name_,
                                       grasp_pose);
  // ROS_DEBUG_STREAM("Object pose is frame " << att.object.header.frame_id << " "
  //                 << att.object.poses[0].position.x << " "
  //                 << att.object.poses[0].position.y << " "
  //                 << att.object.poses[0].position.z);

  // ROS_DEBUG_STREAM("Grasp pose is "
  //                  << grasp.grasp_pose.position.x << " "
  //                  << grasp.grasp_pose.position.y << " "
  //                  << grasp.grasp_pose.position.z);

  // tf::Transform trans = state.getLinkState(att.link_name)->getGlobalLinkTransform();

  // ROS_DEBUG_STREAM("Fingertip pose is "
  //                  << trans.getOrigin().x() << " "
  //                  << trans.getOrigin().y() << " "
  //                  << trans.getOrigin().z());


  geometry_msgs::PoseStamped ps;
  //now we need to get the object pose in terms of the gripper
  cm_.convertPoseGivenWorldTransform(state,
                                     att.link_name,
                                     att.object.header,
                                     att.object.poses[0],
                                     ps);

  att.object.id = "attached_"+att.object.id;
  att.object.header = ps.header;
  att.object.poses[0] = ps.pose;

  ROS_INFO_STREAM("object pose is relative to " + gripper_link_name_ + ": "
                  << ps.pose.position.x << " "
                  << ps.pose.position.y << " "
                  << ps.pose.position.z);

  ROS_INFO_STREAM("Attaching  " << att.object.id << " mode " << att.object.operation.operation);
  planning_scene_diff_.attached_collision_objects.push_back(att);

  attached_object_publisher_.publish(att);
  return true;
}

bool RobotPickPlaceNavigator::detachCollisionObject(const std::string& group_name,
                           const geometry_msgs::PoseStamped& place_pose,
                           const object_manipulation_msgs::Grasp& grasp) {

  ROS_INFO_STREAM("Place pose is " << place_pose.header.frame_id);

  planning_models::KinematicState state(*current_robot_state_);
  tf::Transform place_pose_tf;
  tf::poseMsgToTF(place_pose.pose, place_pose_tf);

  tf::Transform grasp_trans;
  tf::poseMsgToTF(grasp.grasp_pose, grasp_trans);

  place_pose_tf = place_pose_tf*grasp_trans;

  state.updateKinematicStateWithLinkAt(gripper_link_name_,
                                       place_pose_tf);

  if(planning_scene_diff_.attached_collision_objects.size() == 0) {
    ROS_ERROR_STREAM("No attached bodies in scene");
    return false;
  }

  arm_navigation_msgs::CollisionObject& obj =  planning_scene_diff_.attached_collision_objects.front().object;
  const planning_models::KinematicState::AttachedBodyState* att = state.getAttachedBodyState(obj.id);
  if(att == NULL) {
    ROS_ERROR_STREAM("No attached body model " << obj.id << " in attached");
    return false;
  }

  arm_navigation_msgs::AttachedCollisionObject att_publish;
  att_publish.object.id = obj.id;
  att_publish.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  attached_object_publisher_.publish(att_publish);

  obj.header.frame_id = cm_.getWorldFrameId();
  tf::poseTFToMsg(att->getGlobalCollisionBodyTransforms()[0], obj.poses[0]);
  obj.id = current_grasped_object_name_[group_name];
  planning_scene_diff_.collision_objects.push_back(obj);
  planning_scene_diff_.attached_collision_objects.clear();
  return true;
}

void RobotPickPlaceNavigator::callbackPublishMarkers(const ros::TimerEvent &evnt)
{
	for(std::map<std::string,visualization_msgs::Marker>::iterator i = MarkerMap.begin(); i != MarkerMap.end(); i++)
	{
		visualization_msgs::Marker &m = i->second;
		m.header.stamp = ros::Time::now();
		vis_marker_publisher_.publish(m);
	}
}

void RobotPickPlaceNavigator::collisionObjToMarker(const arm_navigation_msgs::CollisionObject &obj,visualization_msgs::Marker &marker)
{
	// only converts to a sphere marker for now
	const arm_navigation_msgs::Shape &shape = obj.shapes[0];

	switch(shape.type)
	{
		case arm_navigation_msgs::Shape::SPHERE:

			marker.header.frame_id = obj.header.frame_id;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.pose = obj.poses[0];
			marker.scale.x = 2*shape.dimensions[0];// 2 x radius
			marker.scale.y =2*shape.dimensions[0];
			marker.scale.z =2*shape.dimensions[0];
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			break;

		default:

			break;

	}

}

void RobotPickPlaceNavigator::startCycleTimer() {
  execution_duration_ = ros::WallDuration(0.0);
  perception_duration_ = ros::WallDuration(0.0);
  planning_scene_duration_ = ros::WallDuration(0.0);
  motion_planning_duration_ = ros::WallDuration(0.0);
  trajectory_filtering_duration_ = ros::WallDuration(0.0);
  grasp_planning_duration_ = ros::WallDuration(0.0);
  cycle_start_time_ = ros::WallTime::now();
}

void RobotPickPlaceNavigator::printTiming()
{
    ros::WallDuration dur = ros::WallTime::now()-cycle_start_time_;
    ROS_INFO_STREAM("Cycle took " << dur.toSec() << " processing " << (dur-execution_duration_).toSec() << " execution " << execution_duration_.toSec());
    ROS_INFO_STREAM("Perception " << perception_duration_.toSec());
    ROS_INFO_STREAM("Planning scene " << planning_scene_duration_.toSec());
    ROS_INFO_STREAM("Planning time " << motion_planning_duration_.toSec());
    ROS_INFO_STREAM("Filtering time " << trajectory_filtering_duration_.toSec());
    ROS_INFO_STREAM("Grasp planning time " << grasp_planning_duration_.toSec());
  }

std::string RobotPickPlaceNavigator::makeCollisionObjectNameFromModelId(unsigned int model_id)
{
  std::stringstream object_id;
  object_id << "object_" << model_id;
  return object_id.str();
}

const arm_navigation_msgs::CollisionObject* RobotPickPlaceNavigator::getCollisionObject(unsigned int model_id)
{
    std::string name = makeCollisionObjectNameFromModelId(model_id);
    for(unsigned int i = 0; i < planning_scene_diff_.collision_objects.size(); i++) {
      if(planning_scene_diff_.collision_objects[i].id == name) return &(planning_scene_diff_.collision_objects[i]);
    }
    return NULL;
}

void RobotPickPlaceNavigator::run()
{
	switch(configuration_type_)
	{
	case SETUP_FULL:
		runFullPickPlace();
		break;

	case SETUP_SPHERE_PICK_PLACE:
		runSpherePickPlace();
		break;

	default:
		break;

	}
}

void RobotPickPlaceNavigator::runFullPickPlace()
{
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	srand(time(NULL));

	setup();// full setup
	moveArmToSide();

	while(ros::ok())
	{
	    startCycleTimer();

//	    ROS_INFO_STREAM(NODE_NAME + ": Segmentation and recognition stage started");
//	    if(!segmentAndRecognize())
//	    {
//	      ROS_WARN_STREAM(NODE_NAME<<": Segment and recognized failed");
//	      continue;
//	    }
//	    ROS_INFO_STREAM(NODE_NAME << " Segmentation and recognition stage completed");
//
//	    ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage started");
//	    if(!pickUpSomething(arm_group_name_))
//	    {
//	      ROS_WARN_STREAM(NODE_NAME << ": grasp pickup stage failed");
//	      continue;
//	    }
//	    else
//	    {
//	    	ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage completed");
//	    }
//
//
//	    ROS_INFO_STREAM(NODE_NAME + ": grasp place stage started");
//	    if(!putDownSomething(arm_group_name_))
//	    {
//	      ROS_WARN_STREAM(NODE_NAME << ": grasp place stage failed");
//	    }
//	    else
//	    {
//	    	ROS_INFO_STREAM(NODE_NAME << ": grasp place stage completed");
//	    }
//
//	    if(!moveArmToSide())
//	    {
//	      ROS_WARN_STREAM(NODE_NAME << ": Final side moved failed");
//	      break;
//	    }

		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
		if(!performSegmentation())
		{
		  ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
		  continue;
		}
		ROS_INFO_STREAM(NODE_NAME << " Segmentation stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": Recognition stage started");
		if(!performRecognition())
		{
		  ROS_WARN_STREAM(NODE_NAME << ": Recognition stage failed");
		  continue;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": Recognition stage completed");
		}

		ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage started");
		if(!moveArmThroughPickSequence())
		{
		  ROS_WARN_STREAM(NODE_NAME << ": grasp pickup stage failed");
		  continue;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage completed");
		}

		ROS_INFO_STREAM(NODE_NAME + ": grasp place stage started");
		if(!moveArmThroughPlaceSequence())
		{
			ROS_WARN_STREAM(NODE_NAME << ": grasp place stage failed");
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": grasp place stage completed");
		}

		if(!moveArmToSide())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Final side moved failed");
			break;
		}

	    printTiming();
	  }
}

void RobotPickPlaceNavigator::runSpherePickPlace()
{
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	srand(time(NULL));

	ROS_INFO_STREAM(NODE_NAME<<" Setup stage started");
	setupBallPickingDemo();
	ROS_INFO_STREAM(NODE_NAME<<" Setup stage completed");

	moveArmToSide();

	while(ros::ok())
	{
	    startCycleTimer();

		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
		if(!performSegmentation() || !performSphereSegmentation())
		{
		  ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
		  continue;
		}
		ROS_INFO_STREAM(NODE_NAME << " Segmentation stage completed");

		ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage started");
		if(!moveArmThroughPickSequence())
		{
		  ROS_WARN_STREAM(NODE_NAME << ": grasp pickup stage failed");
		  continue;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage completed");
		}

		ROS_INFO_STREAM(NODE_NAME + ": grasp place stage started");
		if(!moveArmThroughPlaceSequence())
		{
			ROS_WARN_STREAM(NODE_NAME << ": grasp place stage failed");
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": grasp place stage completed");
		}

		if(!moveArmToSide())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Final side moved failed");
			break;
		}

	    printTiming();
	  }
}

void RobotPickPlaceNavigator::GoalLocation::generateNextLocationCandidates(
		std::vector<geometry_msgs::PoseStamped> &placePoses)
{
		switch(NextLocationGenMode)
		{
		case GoalLocation::FIXED:
			createCandidatePosesByRotation(GoalTransform,NumGoalCandidates,Axis,placePoses);
			break;

		case GoalLocation::CIRCULAR_ARRANGEMENT:
			generateNextLocationCircularMode(placePoses);
			break;

		case GoalLocation::SPIRAL_ARRANGEMENT:
			generateNextLocationSpiralMode(placePoses);
			break;

		case GoalLocation::SQUARE_ARRANGEMENT:
			generateNextLocationSquaredMode(placePoses);
			break;

		case GoalLocation::SHUFFLE:
			generateNextLocationShuffleMode(placePoses);
			break;

		default:
			generateNextLocationShuffleMode(placePoses);
			break;
		}
}

void RobotPickPlaceNavigator::GoalLocation::generateNextLocationShuffleMode(
		std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	// previos pose
	tf::Transform &lastTf = previous_locations_.back();

	// next pose
	tf::Transform nextTf;
	if(previous_locations_.size() == 0)
	{
		nextTf = GoalTransform;
	}
	else
	{
		nextTf = tf::Transform(previous_locations_.back());

		// new location variables
		int distanceSegments = 20; // number of possible values between min and max object spacing
		int angleSegments = 8; // number of possible values between 0 and 2pi
		int randVal;
		double distance;// meters
		double angle,angleMin = 0,angleMax = 2*M_PI; // radians
		double ratio;

		// creating new location relative to the last one
		int maxIterations = 100;
		int iter = 0;
		while(iter < maxIterations)
		{
			iter++;

			// computing distance
			randVal = rand()%distanceSegments + 1;
			ratio = (double)randVal/(double)distanceSegments;
			distance = MinObjectSpacing + ratio*(MaxObjectSpacing - MinObjectSpacing);
			tf::Vector3 trans = tf::Vector3(distance,0.0f,0.0f);

			// computing angle
			randVal = rand()%angleSegments + 1;
			ratio = (double)randVal/(double)angleSegments;
			angle = angleMin + ratio*(angleMax - angleMin);
			tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),angle);

			// computing next pose by rotating and translating from last pose
			nextTf = lastTf * tf::Transform(quat,tf::Vector3(0.0f,0.0f,0.0f))*tf::Transform(tf::Quaternion::getIdentity(),trans);

			// checking if located inside place region
			double distFromCenter = (nextTf.getOrigin() - GoalTransform.getOrigin()).length();
			if((distFromCenter + MinObjectSpacing/2) > PlaceRegionRadius) // falls outside place region, try another
			{
				continue;
			}

			// checking for overlaps against objects already in place region
			double distFromObj;
			BOOST_FOREACH(tf::Transform objTf,previous_locations_)
			{
				distFromObj = (objTf.getOrigin() - nextTf.getOrigin()).length();
				if(distFromObj < MinObjectSpacing)// overlap found, try another
				{
					continue;
				}
			}
		}
	}

	// generating candidate poses from next location found
	createCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

	// storing next location
	previous_locations_.push_back(nextTf);
}

void RobotPickPlaceNavigator::GoalLocation::createCandidatePosesByRotation(const tf::Transform &startTrans,int numCandidates,tf::Vector3 axis,
		std::vector<geometry_msgs::PoseStamped> &candidatePoses)
{
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = GoalTransform.frame_id_;

	// rotate about z axis and apply to original goal pose in order to create candidates;
	for(int i = 0; i < numCandidates; i++)
	{
		double ratio = ((double)i)/((double)numCandidates);
		double angle = 2*M_PI*ratio;
		tf::Quaternion q = tf::Quaternion(axis,angle);
		tf::Vector3 p = tf::Vector3(0,0,0);
		tf::Transform candidateTransform = startTrans*tf::Transform(q,p);
		tf::poseTFToMsg(candidateTransform,pose.pose);
		candidatePoses.push_back(pose);
	}
}
