/*
 * RobotNavigator.cpp
 *
 *  Created on: Oct 18, 2012
 *      Author: jnicho
 */

#include <object_manipulation_tools/robot_navigators/RobotNavigator.h>
#include <boost/foreach.hpp>
#include <object_manipulation_tools/manipulation_utils/Utilities.h>

/* continue modifications in move arm
 *
 */
std::string RobotNavigator::NODE_NAME = "robot_pick_place";
std::string RobotNavigator::NAVIGATOR_NAMESPACE = "navigator";
std::string RobotNavigator::MARKER_ARM_LINK = "arm_links";
std::string RobotNavigator::MARKER_ATTACHED_OBJECT = "attached_object";
std::string RobotNavigator::VISUALIZATION_TOPIC = "visualization_markers";

const tf::Transform RobotNavigator::PLACE_RECTIFICATION_TF = tf::Transform(
		tf::Quaternion(tf::Vector3(1.0f,0.0f,0.0f),M_PI),tf::Vector3(0.0f,0.0f,0.0f));

// global variables
const double GRAP_ACTION_TIMEOUT = 4.0f;

void RobotNavigator::fetchParameters(std::string nameSpace)
{
	ros::NodeHandle nh;
	using namespace RobotNavigatorParameters;

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

	ros::param::param(nameSpace + "/" + PARAM_PICK_APPROACH_DISTANCE,pick_approach_distance_,DF_PICK_APPROACH_DISTANCE);
	ros::param::param(nameSpace + "/" + PARAM_PLACE_APPROACH_DISTANCE,place_approach_distance_,DF_PLACE_APPROACH_DISTANCE);
	ros::param::param(nameSpace + "/" + PARAM_PLACE_RETREAT_DISTANCE,place_retreat_distance_,DF_PLACE_RETREAT_DISTANCE);
}

RobotNavigator::RobotNavigator()
: cm_("robot_description"),
  current_robot_state_(NULL),
  pick_approach_distance_(RobotNavigatorParameters::DF_PICK_APPROACH_DISTANCE),
  place_approach_distance_(RobotNavigatorParameters::DF_PLACE_APPROACH_DISTANCE),
  place_retreat_distance_(RobotNavigatorParameters::DF_PLACE_RETREAT_DISTANCE)

{
	ros::NodeHandle nh;

	// initializing name spaces global strings
	NODE_NAME = ros::this_node::getName();
	NAVIGATOR_NAMESPACE = NODE_NAME + "/" + NAVIGATOR_NAMESPACE;
	VISUALIZATION_TOPIC = NODE_NAME + "/" + VISUALIZATION_TOPIC;
}

RobotNavigator::~RobotNavigator()
{

}

void RobotNavigator::setup()
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
		// perception
		seg_srv_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(segmentation_service_, true);
		rec_srv_ = nh.serviceClient<tabletop_object_detector::TabletopObjectRecognition>(
				recognition_service_, true);
		object_database_model_mesh_client_ = nh.serviceClient<household_objects_database_msgs::GetModelMesh>(
				mesh_database_service_, true);
		object_database_model_description_client_ = nh.serviceClient<household_objects_database_msgs::GetModelDescription>(
				model_database_service_, true);

		// path and grasp plannig
		grasp_planning_client = nh.serviceClient<object_manipulation_msgs::GraspPlanning>(grasp_planning_service_, false);
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
		marker_publisher_ = nh.advertise<visualization_msgs::Marker> (VISUALIZATION_TOPIC, 128);
		attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);

		// setting up timer obj
		marker_pub_timer_ = nh.createTimer(ros::Duration(0.4f),&RobotNavigator::callbackPublishMarkers,this);

		ROS_INFO_STREAM(NODE_NAME<<": Setting up dynamic libraries");

		// others
		grasp_tester_ = GraspTesterPtr(new GraspSequenceValidator(&cm_, ik_plugin_name_));
		place_tester_ = PlaceSequencePtr(new PlaceSequenceValidator(&cm_, ik_plugin_name_));

		trajectories_finished_function_ = boost::bind(&RobotNavigator::trajectoryFinishedCallback, this, true,_1);
		grasp_action_finished_function_ = boost::bind(&RobotNavigator::trajectoryFinishedCallback, this, false,_1);
		ROS_INFO_STREAM(NODE_NAME<<": Finished setup");
	}

	ROS_INFO_STREAM(NODE_NAME<<" Setting up grasp planning data");
	{
		// storing grasp pickup goal to be used later during pick move sequence execution
		grasp_pickup_goal_.arm_name = arm_group_name_;
		grasp_pickup_goal_.lift.direction.header.frame_id = cm_.getWorldFrameId();
		grasp_pickup_goal_.lift.direction.vector.z = 1.0;
		grasp_pickup_goal_.lift.desired_distance = pick_approach_distance_;
		grasp_pickup_goal_.lift.min_distance = pick_approach_distance_;
		grasp_pickup_goal_.lift.direction.header.frame_id = cm_.getWorldFrameId();
		grasp_pickup_goal_.allow_gripper_support_collision = true;
		grasp_pickup_goal_.collision_support_surface_name = "table";

		// populate grasp place goal
		grasp_place_goal_.arm_name = arm_group_name_;
		grasp_place_goal_.desired_retreat_distance = place_retreat_distance_;
		grasp_place_goal_.min_retreat_distance = place_retreat_distance_;
		grasp_place_goal_.approach.desired_distance = place_approach_distance_;
		grasp_place_goal_.approach.min_distance = place_approach_distance_;
		grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
		grasp_place_goal_.approach.direction.vector.x = 0.0;
		grasp_place_goal_.approach.direction.vector.y = 0.0;
		grasp_place_goal_.approach.direction.vector.z = -1.0;
		grasp_place_goal_.allow_gripper_support_collision = true;
		grasp_place_goal_.collision_support_surface_name = "table";
		grasp_place_goal_.place_padding = .02;
	}
}

bool RobotNavigator::trajectoryFinishedCallback(bool storeLastTraj,TrajectoryExecutionDataVector tedv)
{
	ROS_INFO_STREAM(NODE_NAME<<":Trajectory execution result flag: "<<tedv.back().result_);

	// combining trajectory state flags;
	trajectories_succeeded_ = (tedv.back().result_ == SUCCEEDED
							  || tedv.back().result_ == HANDLER_REPORTS_FAILURE_BUT_OK
							  || tedv.back().result_ == HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH);

	if(storeLastTraj)
	{
		if(tedv.size()==0)
		{
			ROS_ERROR_STREAM(NODE_NAME <<": trajectory finished callback received empty vector");
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME<<": trajectory finished callback storing last trajectory");
			last_trajectory_execution_data_vector_ = tedv;
		}
	}
	else
	{
		ROS_INFO_STREAM(NODE_NAME<<": trajectory finished callback will not store last trajectory");
	}


	ROS_INFO_STREAM(NODE_NAME<<": Trajectory finished callback notifying all awaiting threads");
	execution_completed_.notify_all();
	return true;
}

void RobotNavigator::revertPlanningScene()
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

std::vector<std::string> RobotNavigator::getJointNames(const std::string& group) {
  if(cm_.getKinematicModel()->getModelGroup(group) == NULL) {
    std::vector<std::string> names;
    return names;
  }
  return cm_.getKinematicModel()->getModelGroup(group)->getJointModelNames();
}

void RobotNavigator::updateCurrentJointStateToLastTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj)
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

bool RobotNavigator::updateChangesToPlanningScene()
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

bool RobotNavigator::moveArm(const std::string& group_name,const std::vector<double>& joint_positions)
{

  // requesting path plan
  ros::WallTime start_time = ros::WallTime::now();
  arm_navigation_msgs::GetMotionPlan::Request plan_req;
  arm_navigation_msgs::GetMotionPlan::Response plan_res;

  plan_req.motion_plan_request.allowed_planning_time = ros::Duration(20.0f);
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
    joint_constraints[i].tolerance_above = 0.2f;
    joint_constraints[i].tolerance_below = 0.2f;
  }

  if(!planning_service_client_.call(plan_req, plan_res))
  {
    ROS_WARN_STREAM(NODE_NAME<<": Planner service call failed");
    return false;
  }

  if(plan_res.error_code.val != plan_res.error_code.SUCCESS)
  {
    ROS_WARN_STREAM(NODE_NAME<<": Planner failed, error code:"<<plan_res.error_code.val);
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

  ROS_INFO_STREAM("Arm Trajectory in progress");
  trajectory_execution_monitor_.executeTrajectories(ter_reqs,trajectories_finished_function_);

  boost::unique_lock<boost::mutex> lock(execution_mutex_);
  execution_completed_.wait(lock);
  execution_duration_ += (ros::WallTime::now()-start_execution);

  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  if(trajectories_succeeded_)
  {
    error_code.val = error_code.SUCCESS;
    ROS_INFO_STREAM("Trajectory execution has succeeded");
  }
  else
  {
    error_code.val = error_code.PLANNING_FAILED;
    ROS_ERROR_STREAM("Trajectory execution has failed");
  }

  return trajectories_succeeded_;
}

bool RobotNavigator::validateJointTrajectory(trajectory_msgs::JointTrajectory &jt)
{
	const ros::Duration timeIncrement(1.0f);
	ros::Duration currentTime = timeIncrement ;

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

		p.time_from_start = currentTime;
		currentTime = currentTime+ timeIncrement;
	}

	return true;
}

void RobotNavigator::printJointTrajectory(const trajectory_msgs::JointTrajectory &jt)
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

bool RobotNavigator::performTrajectoryFiltering(const std::string& group_name,trajectory_msgs::JointTrajectory& jt)
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

//    ROS_INFO_STREAM(NODE_NAME<<": Populating joint names: " << jt.joint_names[i]
//                    << ", jvals: " << jvals[jt.joint_names[i]]
//                    << ", jt.points: " << jt.points[0].positions[i]);
  }

  planning_models::KinematicState state(*current_robot_state_);
  state.setKinematicState(jvals);

  filter_req.trajectory = jt;
  planning_environment::convertKinematicStateToRobotState(state,ros::Time::now(),
		  cm_.getWorldFrameId(),filter_req.start_state);
  filter_req.group_name = group_name;

  if(!trajectory_filter_service_client_.call(filter_req, filter_res))
  {
    ROS_WARN_STREAM(NODE_NAME<<": Fast Filter service call failed");
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


  return true;
}

bool RobotNavigator::moveArmToSide()
{
    updateChangesToPlanningScene();

	std::vector<double> joint_angles;
	joint_angles.push_back(-1.0410828590393066);
	joint_angles.push_back(0.46065822721369176);
	joint_angles.push_back(2.4644586717834467);
	joint_angles.push_back(0.49449136755439443);
	joint_angles.push_back(-0.2900361153401066);
	joint_angles.push_back(1.4113548618662812);
	joint_angles.push_back(2.3286899342716625);

    return moveArm(arm_group_name_,joint_angles);
}

void RobotNavigator::addDetectedTableToLocalPlanningScene(const tabletop_object_detector::Table &table)
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

void RobotNavigator::addDetectedObjectToLocalPlanningScene(arm_navigation_msgs::CollisionObject &obj)
{
  obj.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  planning_scene_diff_.collision_objects.push_back(obj);
}

void RobotNavigator::addDetectedObjectToLocalPlanningScene(const household_objects_database_msgs::DatabaseModelPoseList& model)
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
  getMeshFromDatabasePose(model.model_list[0],obj,obj_world);

  ROS_INFO_STREAM(NODE_NAME<<": Pushing back " << obj.id << " frame " << obj.header.frame_id);
  planning_scene_diff_.collision_objects.push_back(obj);
}

bool RobotNavigator::performSegmentation()
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
	addDetectedTableToLocalPlanningScene(segmentation_srv.response.table);

	// ===================================== printing completion info message =====================================
	ROS_INFO_STREAM(NODE_NAME<<": Segmentation service succeeded. Detected "<<(int)segmentation_srv.response.clusters.size()<<" clusters");

	return true;
}

bool RobotNavigator::performRecognition()
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
    addDetectedObjectToLocalPlanningScene(recognition_srv.response.models[0]);

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

bool RobotNavigator::getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
                             arm_navigation_msgs::CollisionObject& obj,
                             const geometry_msgs::PoseStamped& pose)
{
  household_objects_database_msgs::GetModelMesh::Request req;
  household_objects_database_msgs::GetModelMesh::Response res;

  req.model_id = model_pose.model_id;
  if(!object_database_model_mesh_client_.call(req, res))
  {
    ROS_WARN_STREAM(NODE_NAME<<": Call to objects database for getMesh failed");
    return false;
  }

  if(res.return_code.code != res.return_code.SUCCESS)
  {
    ROS_WARN_STREAM(NODE_NAME<<": Object database gave non-success code " << res.return_code.code << " for model id " << req.model_id);
    return false;
  }

  obj.header = pose.header;
  obj.poses.resize(1);
  obj.shapes.resize(1);

  recognized_obj_pose_map_[obj.id] = pose;

  bool use_cylinder = true;

  if(use_cylinder)
  {
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
  }
  else
  {
    obj.shapes.resize(1);
    obj.shapes[0].type = arm_navigation_msgs::Shape::MESH;
    obj.shapes[0] = res.mesh;
    obj.poses[0] = pose.pose;
  }
  return true;
}

bool RobotNavigator::performPickGraspPlanning()
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
	//request.target.reference_frame_id = segmented_clusters_[0].header.frame_id;
	request.target.cluster = segmented_clusters_[0];
	request.target.reference_frame_id = segmentation_results_.table.pose.header.frame_id;

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
	else
	{
		ROS_INFO_STREAM(NODE_NAME<<": Grasp Planner Srcv returned "<<response.grasps.size()<<" grasp candidates");
	}

	//TODO - actually deal with the different cases here, especially for the cluster planner
	if(request.target.reference_frame_id != recognized_model_description_.name ||
		  request.target.reference_frame_id != modelPose.pose.header.frame_id)
	{
	  ROS_WARN_STREAM("Cluster does not match recognition");
	}

	//  ===================================== storing results =====================================
	/* Storing grasp candidates:
	 * 	Grasp poses return by the service define the location of the tcp in terms of the world.
	 * 	However, planning is done with the assumption that the grasp indicate the location
	 * 	of the wrist relative to the object.
	 */
	grasp_candidates_.assign(response.grasps.begin(),response.grasps.end());

	//  =====================================  grasp planning for pick move ==================================
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

	// updating grasp pickup goal data
	grasp_pickup_goal_.arm_name = arm_group_name_;
	grasp_pickup_goal_.collision_object_name = modelId;
	grasp_pickup_goal_.lift.direction.header.frame_id = cm_.getWorldFrameId();
	grasp_pickup_goal_.target.reference_frame_id = modelId;
	grasp_pickup_goal_.target.cluster = segmented_clusters_[0];
	grasp_pickup_goal_.target.potential_models.push_back(modelPose);

	// generating grasp pick sequence
	updateChangesToPlanningScene();
	grasp_pick_sequence_.clear();
	std::vector<object_manipulation_msgs::Grasp> valid_grasps;
	if(!createPickMoveSequence(grasp_pickup_goal_,grasp_candidates_,grasp_pick_sequence_,valid_grasps))
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp pick sequence");
		return false;
	}
	grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());
	return true;
}

bool RobotNavigator::performPlaceGraspPlanning()
{
	using namespace manipulation_utils;

	//	updating grasp place goal data
	grasp_place_goal_.arm_name = arm_group_name_;
	grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
	grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];

	// creating candidate grasp place poses
	std::vector<geometry_msgs::PoseStamped> placePoses;
	if(!createCandidateGoalPoses(placePoses))
	{
		ROS_ERROR_STREAM(NODE_NAME<<": Could not create valid place poses");
		return false;
	}

	// finding valid grasp place sequence
	updateChangesToPlanningScene();
	bool found_valid = false;
	geometry_msgs::Pose valid_grasp_place_pose; // will keep only valid grasp pick sequence which grasp yields a valid place sequence;
	std::vector<object_manipulation_msgs::Grasp> valid_grasps;
	std::vector<object_manipulator::GraspExecutionInfo> valid_pick_sequence;
	std::vector<object_manipulator::PlaceExecutionInfo> valid_place_sequence;

	// finding pose of wrist relative to object
	tf::StampedTransform wrist_in_tcp_tf, wrist_in_obj_tf;// wrist pose relative to gripper
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);
	for(std::size_t i = 0; i < grasp_candidates_.size(); i++)
	{
		// storing tcp to object pose in grasp place goal
		tf::poseMsgToTF(grasp_candidates_[i].grasp_pose,wrist_in_obj_tf);
		tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),grasp_place_goal_.grasp.grasp_pose);
		manipulation_utils::rectifyPoseZDirection(grasp_place_goal_.grasp.grasp_pose,
				PLACE_RECTIFICATION_TF,grasp_place_goal_.grasp.grasp_pose);
		if(createPlaceMoveSequence(grasp_place_goal_,placePoses,valid_place_sequence))
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
		// storing valid pick sequences
		grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());
		grasp_pick_sequence_.assign(valid_pick_sequence.begin(),valid_pick_sequence.end());
		grasp_place_goal_.grasp.grasp_pose = valid_grasp_place_pose;
	}


	//  ===================================== updating local planning scene =====================================
	// updating gripper (not sure if this is necessary)
	tf::Transform first_grasp_in_world_tf;
	tf::poseMsgToTF(grasp_candidates_[0].grasp_pose, first_grasp_in_world_tf);
	planning_models::KinematicState state(*current_robot_state_);
	state.updateKinematicStateWithLinkAt(gripper_link_name_, first_grasp_in_world_tf);

	return true;
}

bool RobotNavigator::performGraspPlanning()
{

	return performPickGraspPlanning() && performPlaceGraspPlanning();
//	//  ===================================== saving current time stamp =====================================
//	ros::WallTime start_time = ros::WallTime::now();
//
//	//  ===================================== clearing results from last call =====================================
//	grasp_pickup_goal_.target.potential_models.clear();
//	grasp_candidates_.clear();
//
//	//  ===================================== calling service =====================================
//	// checking available recognized models
//	if((recognized_models_.size() == 0) || (recognized_models_[0].model_list.size() == 0))
//	{
//	  return false;
//	}
//
//	// populating grasp plan request/response
//	household_objects_database_msgs::DatabaseModelPose modelPose = recognized_models_[0].model_list[0];
//	object_manipulation_msgs::GraspPlanning::Request request;
//	object_manipulation_msgs::GraspPlanning::Response response;
//	std::string modelId = makeCollisionObjectNameFromModelId(modelPose.model_id);
//
//	//modelPose.pose = geometry_msgs::PoseStamped();
//	//modelPose.pose.pose.orientation.w = 1.0;
//	modelPose.pose = recognized_obj_pose_map_[modelId];
//	modelPose.pose.header.frame_id = recognized_model_description_.name; // should be updated during recognition stage
//	modelPose.pose.header.stamp = ros::Time::now();
//	request.arm_name = arm_group_name_;
//	request.target.potential_models.push_back(modelPose);
//	request.target.reference_frame_id = segmented_clusters_[0].header.frame_id;
//	request.target.cluster = segmented_clusters_[0];
//
//	bool success = grasp_planning_client.call(request, response);
//
//	// ===================================== checking results ========================================
//	if(!success)
//	{
//		ROS_WARN_STREAM(NODE_NAME<<": grasp planning call unsuccessful, exiting");
//		return false;
//	}
//
//	if(response.error_code.value != response.error_code.SUCCESS)
//	{
//		ROS_WARN_STREAM(NODE_NAME<<": grasp planning call returned error code, exiting " << response.error_code.value);
//		return false;
//	}
//
//	if(response.grasps.size() == 0)
//	{
//		ROS_WARN_STREAM(NODE_NAME<<": No grasps returned in response");
//		return false;
//	}
//
//	//TODO - actually deal with the different cases here, especially for the cluster planner
//	if(request.target.reference_frame_id != recognized_model_description_.name ||
//		  request.target.reference_frame_id != modelPose.pose.header.frame_id)
//	{
//	  ROS_WARN_STREAM("Cluster does not match recognition");
//	}
//
//	//  ===================================== storing results =====================================
//	/* Storing grasp candidates:
//	 * 	Grasp poses return by the service define the location of the tcp in terms of the world.
//	 * 	However, planning is done with the assumption that the grasp indicate the location
//	 * 	of the wrist relative to the object.
//	 */
//	grasp_candidates_.assign(response.grasps.begin(),response.grasps.end());
//
//	//  =====================================  grasp planning for pick move ==================================
//	// instantiating needed transforms and poses
//	tf::Transform object_in_world_tf;
//	tf::StampedTransform wristInGripperTcp = tf::StampedTransform();
//	tf::Transform object_in_world_inverse_tf;
//
//	// populating transforms
//	tf::poseMsgToTF(modelPose.pose.pose, object_in_world_tf);
//	object_in_world_inverse_tf = object_in_world_tf.inverse();
//	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wristInGripperTcp);
//
//	// applying transformation to grasp pose so that the arm wrist relative to the object is obtained
//	for(unsigned int i = 0; i < grasp_candidates_.size(); i++)
//	{
//		tf::Transform grasp_in_world_tf;
//		tf::poseMsgToTF(grasp_candidates_[i].grasp_pose, grasp_in_world_tf);
//		tf::poseTFToMsg(object_in_world_inverse_tf*(grasp_in_world_tf*wristInGripperTcp),
//			  grasp_candidates_[i].grasp_pose);
//	}
//
//	// updating grasp pickup goal data
//	grasp_pickup_goal_.arm_name = arm_group_name_;
//	grasp_pickup_goal_.collision_object_name = modelId;
//	grasp_pickup_goal_.lift.direction.header.frame_id = cm_.getWorldFrameId();
//	grasp_pickup_goal_.target.reference_frame_id = modelId;
//	grasp_pickup_goal_.target.cluster = segmented_clusters_[0];
//	grasp_pickup_goal_.target.potential_models.push_back(modelPose);
//
//	// generating grasp pick sequence
//	updateChangesToPlanningScene();
//	grasp_pick_sequence_.clear();
//	std::vector<object_manipulation_msgs::Grasp> valid_grasps;
//	if(!createPickMoveSequence(grasp_pickup_goal_,grasp_candidates_,grasp_pick_sequence_,valid_grasps))
//	{
//		ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp pick sequence");
//		return false;
//	}
//	grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());
//
//
//	//  ==================================  grasp planning for place move ================================
//	//	updating grasp place goal data
//	grasp_place_goal_.arm_name = arm_group_name_;
//	grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
//	grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];
//
//	// creating candidate grasp place poses
//	std::vector<geometry_msgs::PoseStamped> placePoses;
//	if(!createCandidateGoalPoses(placePoses))
//	{
//		ROS_ERROR_STREAM(NODE_NAME<<": Could not create valid place poses");
//		return false;
//	}
//
//	// finding valid grasp place sequence
//	updateChangesToPlanningScene();
//	bool found_valid = false;
//	valid_grasps.clear(); // will keep only valid grasp pick sequence which grasp yields a valid place sequence;
//	geometry_msgs::Pose valid_grasp_place_pose;
//	std::vector<object_manipulator::GraspExecutionInfo> valid_pick_sequence;
//	std::vector<object_manipulator::PlaceExecutionInfo> valid_place_sequence;
//
//	// finding pose of wrist relative to object
//	tf::StampedTransform wrist_in_tcp_tf, wrist_in_obj_tf;// wrist pose relative to gripper
//	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);
//	for(std::size_t i = 0; i < grasp_candidates_.size(); i++)
//	{
//		tf::poseMsgToTF(grasp_candidates_[i].grasp_pose,wrist_in_obj_tf);
//		tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),grasp_place_goal_.grasp.grasp_pose);
//		if(createPlaceMoveSequence(grasp_place_goal_,placePoses,valid_place_sequence))
//		{
//			if(!found_valid)
//			{
//				// storing first valid
//				grasp_place_sequence_.assign(valid_place_sequence.begin(),valid_place_sequence.end());
//				valid_grasp_place_pose = grasp_place_goal_.grasp.grasp_pose;
//			}
//
//			found_valid = true;
//			valid_grasps.push_back(grasp_candidates_[i]);
//			valid_pick_sequence.push_back(grasp_pick_sequence_[i]);
//			grasp_place_goal_.grasp.grasp_pose = valid_grasp_place_pose;
//		}
//	}
//
//	if(!found_valid)
//	{
//		ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp place sequence");
//		return false;
//	}
//	else
//	{
//		// storing valid pick sequences
//		grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());
//		grasp_pick_sequence_.assign(valid_pick_sequence.begin(),valid_pick_sequence.end());
//	}
//
//
//	//  ===================================== updating local planning scene =====================================
//	// updating gripper (not sure if this is necessary)
//	tf::Transform first_grasp_in_world_tf;
//	tf::poseMsgToTF(response.grasps[0].grasp_pose, first_grasp_in_world_tf);
//	planning_models::KinematicState state(*current_robot_state_);
//	state.updateKinematicStateWithLinkAt(gripper_link_name_, first_grasp_in_world_tf);
//
//	// ===================================== printing completion info message =====================================
//	ROS_INFO_STREAM(NODE_NAME<<": Grasp is " << response.grasps[0].grasp_pose.position.x << " "
//			  << response.grasps[0].grasp_pose.position.y << " "
//			  << response.grasps[0].grasp_pose.position.z);
//
//	return true;
}

bool RobotNavigator::createPickMoveSequence(
		const object_manipulation_msgs::PickupGoal &pickupGoal,
		const std::vector<object_manipulation_msgs::Grasp> &grasps_candidates,
		std::vector<object_manipulator::GraspExecutionInfo> &grasp_sequence,
		std::vector<object_manipulation_msgs::Grasp> &valid_grasps)
{
	// results data
	std::vector<object_manipulator::GraspExecutionInfo> candidate_sequence;

    ROS_INFO_STREAM(NODE_NAME<<": Evaluating grasps with grasp tester");
	planning_models::KinematicState kinematicState(*current_robot_state_);
	grasp_tester_->setPlanningSceneState(&kinematicState);
	grasp_tester_->testGrasps(pickupGoal,grasps_candidates,candidate_sequence,true);

	// keeping successful grasp candidates
	bool found_valid = false;
	for(std::size_t i = 0; i < candidate_sequence.size(); i++)
	{
		if(candidate_sequence[i].result_.result_code == object_manipulation_msgs::GraspResult::SUCCESS)
		{
			grasp_sequence.push_back(candidate_sequence[i]);
			valid_grasps.push_back(grasp_candidates_[i]);
			found_valid = true;
		}
	}

    ROS_INFO_STREAM(NODE_NAME<<": Returned "<< grasp_sequence.size() <<" grasps candidates ");
    return found_valid;
}

bool RobotNavigator::createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &placeGoal,
				const std::vector<geometry_msgs::PoseStamped> &place_poses,
				std::vector<object_manipulator::PlaceExecutionInfo> &place_sequence)
{
	// result data
	std::vector<object_manipulator::PlaceExecutionInfo> candidate_sequence;

	// find transform of wrist relative to the gripper's tcp
	tf::StampedTransform gripperTcpToWrist = tf::StampedTransform();
	planning_models::KinematicState state(*current_robot_state_);
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),gripperTcpToWrist);
	place_tester_->setTcpToWristTransform(gripperTcpToWrist);
	place_tester_->setPlanningSceneState(&state);
	place_tester_->testPlaces(placeGoal, place_poses, candidate_sequence, true);

	// keeping successful place candidates
	bool found_valid = false;
	std::vector<object_manipulator::PlaceExecutionInfo>::iterator i;
	for(i = candidate_sequence.begin(); i != candidate_sequence.end();i++)
	{
		if(i->result_.result_code == object_manipulation_msgs::PlaceLocationResult::SUCCESS)
		{
			place_sequence.push_back(*i);
			found_valid = true;
		}
	}

	return found_valid;
}

bool RobotNavigator::moveArmThroughPickSequence()
{

	using namespace manipulation_utils;
	// pushing local changes to planning scene
	updateChangesToPlanningScene();

	// grasp planning
	bool success = performGraspPlanning();
	if(!success)
	{
		ROS_INFO_STREAM(NODE_NAME<<": Grasp Pick attempt aborted");
		return false;
	}

	// transform for recalculation of results more than one grasp attempt is made
	tf::StampedTransform wrist_in_tcp_tf;// wrist pose relative to gripper
	tf::Transform wrist_in_obj_tf;
	_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);

	// try each successful grasp
	success = false;
	int counter = 0; // index to grasp array
	BOOST_FOREACH(object_manipulator::GraspExecutionInfo graspMoves,grasp_pick_sequence_)
	{
		ROS_INFO_STREAM(NODE_NAME<<": Attempting Grasp Pick sequence");
		success = attemptGraspSequence(arm_group_name_,graspMoves);
		if(!success)
		{
			if(++counter == (int)grasp_pick_sequence_.size()) return false; // no more valid sequences

			tf::poseMsgToTF(grasp_candidates_[counter].grasp_pose,wrist_in_obj_tf);
			tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),grasp_place_goal_.grasp.grasp_pose);
			manipulation_utils::rectifyPoseZDirection(grasp_place_goal_.grasp.grasp_pose,
							PLACE_RECTIFICATION_TF,grasp_place_goal_.grasp.grasp_pose);

			// recomputing candidate grasp place poses
			std::vector<geometry_msgs::PoseStamped> placePoses;
			if(!createCandidateGoalPoses(placePoses) || !createPlaceMoveSequence(grasp_place_goal_,placePoses,grasp_place_sequence_))
			{
				ROS_ERROR_STREAM(NODE_NAME<<": Could not grasp place solution for current grasp pick");
				return false;
			}

			ROS_INFO_STREAM(NODE_NAME<<": Grasp pick move failed, try next");
			//return false;
			continue;
		}
		else
		{
		  ROS_INFO_STREAM(NODE_NAME<<": Grasp pick move succeeded");
		}

		// updating attached object marker pose
		if(hasMarker(MARKER_ATTACHED_OBJECT))
		{
			visualization_msgs::Marker &m = getMarker(MARKER_ATTACHED_OBJECT);
			tf::poseTFToMsg(wrist_in_obj_tf.inverse(),m.pose);
			m.header.frame_id = gripper_link_name_;
			addMarker(MARKER_ATTACHED_OBJECT,m);
		}

		counter++;
	}

	return success;
}

bool RobotNavigator::moveArmThroughPlaceSequence()
{
	// resetting scene
	updateChangesToPlanningScene();

	// starting timer
	ros::WallTime start_time = ros::WallTime::now();

	// checking grasp
	if(!object_in_hand_map_[arm_group_name_])
	{
		ROS_WARN_STREAM(NODE_NAME<<": No object in hand reported, continuing");
		//return false;
	}

//	updating grasp place data
	grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];

	// try each place sequence candidate
	bool success = false;
	BOOST_FOREACH(object_manipulator::PlaceExecutionInfo placeMove,grasp_place_sequence_)
	{
		success = attemptPlaceSequence(arm_group_name_,placeMove);
		if(success)
		{
			ROS_INFO_STREAM(NODE_NAME<<": Grasp place move succeeded");
		}
		else
		{
			ROS_ERROR_STREAM(NODE_NAME<<"Grasp place move failed, , trying next");
			//return false;
			continue;
		}

		break;
	}

	return success;
}

void RobotNavigator::addMarker(std::string name,visualization_msgs::Marker &marker)
{
	if(marker_map_.count(name) > 0)
	{
		marker.id = marker_map_[name].id;
		marker_map_[name] = marker;
	}
	else
	{
		marker.id = marker_map_.size();
		marker_map_.insert(std::make_pair(name,marker));
	}
}

void RobotNavigator::addMarker(std::string name,visualization_msgs::MarkerArray &markers)
{
	std::stringstream nameStream;
	for(unsigned int i = 0; i < markers.markers.size(); i++)
	{
		nameStream<<name<<i;
		addMarker(nameStream.str(),markers.markers[i]);
		nameStream.str("");
	}
}

bool RobotNavigator::hasMarker(std::string name)
{
	if(marker_map_.count(name) > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

visualization_msgs::Marker& RobotNavigator::getMarker(std::string name)
{
	return marker_map_[name];
}

void RobotNavigator::callbackPublishMarkers(const ros::TimerEvent &evnt)
{
	for(std::map<std::string,visualization_msgs::Marker>::iterator i = marker_map_.begin(); i != marker_map_.end(); i++)
	{
		visualization_msgs::Marker &m = i->second;
		m.header.stamp = ros::Time::now();
		marker_publisher_.publish(m);
	}
}

void RobotNavigator::attachCollisionObjectCallback(const std::string& group_name)
{
  attachCollisionObject(group_name,
                        current_grasped_object_name_[group_name],
                        current_grasp_map_[group_name]);
  object_in_hand_map_[group_name] = true;
}

void RobotNavigator::detachCollisionObjectCallback(const std::string& group_name)
{
  detachCollisionObject(group_name,
                        current_place_location_,
                        current_grasp_map_[group_name]);
  object_in_hand_map_[group_name] = false;
  current_grasped_object_name_.erase(group_name);
  current_grasp_map_.erase(group_name);
}

bool RobotNavigator::attemptGraspSequence(const std::string& group_name,
                          const object_manipulator::GraspExecutionInfo& gei,
                          bool performRecoveryMove)
{

  std::vector<std::string> segment_names;
  std::vector< boost::function<bool(TrajectoryExecutionDataVector)> > callbacks;

  // commanding gripper release
  std::vector<TrajectoryExecutionRequest> ter_reqs;
  TrajectoryExecutionRequest gripper_ter;
  gripper_ter.group_name_ = gripper_group_name_;
  gripper_ter.controller_name_ = grasp_action_service_;
  gripper_ter.trajectory_ = getGripperTrajectory(object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE);
  gripper_ter.failure_ok_ = false;
  gripper_ter.test_for_close_enough_ = false;
  //gripper_ter.monitor_overshoot_ = false;
  ter_reqs.push_back(gripper_ter);

  ros::WallTime start_execution = ros::WallTime::now();
  ROS_INFO_STREAM(NODE_NAME << ": Gripper Release trajectory in progress");
  //trajectory_execution_monitor_.executeTrajectories(ter_reqs,trajectories_finished_function_);
  trajectory_execution_monitor_.executeTrajectories(ter_reqs,grasp_action_finished_function_);
  {
    boost::unique_lock<boost::mutex> lock(execution_mutex_);
    execution_completed_.wait(lock);
  }

  execution_duration_ += (ros::WallTime::now()-start_execution);
  ROS_INFO_STREAM( NODE_NAME << ": Gripper Release trajectory completed");
  ter_reqs.clear();

  // moving arm from current configuration to pre-grasp configuration
  updateCurrentJointStateToLastTrajectoryPoint(last_trajectory_execution_data_vector_.back().recorded_trajectory_);
  ROS_INFO_STREAM(NODE_NAME << ": Moving arm to beginning of pick trajectory");
  if(!moveArm(group_name, gei.approach_trajectory_.points[0].positions))
  {
    return false;
  }
  ROS_INFO_STREAM(NODE_NAME << ": Move completed");
  trajectories_succeeded_ = false;

  // setting up gripper pre-grasp
  gripper_ter.trajectory_ = getGripperTrajectory(object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP);
  validateJointTrajectory(gripper_ter.trajectory_);
  ter_reqs.push_back(gripper_ter);
  segment_names.push_back("Pre-grasp");
  callbacks.push_back(grasp_action_finished_function_);

  // setting up approach move
  TrajectoryExecutionRequest arm_ter;
  arm_ter.group_name_ = group_name;
  arm_ter.controller_name_ = arm_controller_handler_->getControllerName();
  arm_ter.trajectory_ = gei.approach_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  performTrajectoryFiltering(arm_ter.group_name_, arm_ter.trajectory_);
  arm_ter.test_for_close_enough_ = false;
  arm_ter.monitor_overshoot_ = false;
  arm_ter.min_overshoot_time_ = ros::Duration(0.0f);
  arm_ter.max_overshoot_time_ = ros::Duration(5.0f);
  arm_ter.failure_ok_ = false;
  arm_ter.max_joint_distance_ = .1;
  arm_ter.failure_time_factor_ = 1000;
  arm_ter.callback_function_ = boost::bind(&RobotNavigator::attachCollisionObjectCallback, this, _1);
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("Approach");
  callbacks.push_back(trajectories_finished_function_);

  // setting up gripper grasp
  gripper_ter.trajectory_ = getGripperTrajectory(object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP);
  validateJointTrajectory(gripper_ter.trajectory_);
  ter_reqs.push_back(gripper_ter);
  segment_names.push_back("Grasp");
  callbacks.push_back(grasp_action_finished_function_);

  // updating attached  marker operation
  if(hasMarker(MARKER_ATTACHED_OBJECT))
  {
	  visualization_msgs::Marker &m = getMarker(MARKER_ATTACHED_OBJECT);
	  m.action = visualization_msgs::Marker::ADD;
	  addMarker(MARKER_ATTACHED_OBJECT,m);
  }

  //setting up lift move
  arm_ter.trajectory_ = gei.lift_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  performTrajectoryFiltering(group_name, arm_ter.trajectory_);
  arm_ter.callback_function_ = NULL;
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("Lift");
  callbacks.push_back(trajectories_finished_function_);

  /*
   * executing all trajectories
   * currently this is the only way to execute multiple trajectories in sequence, since the executeTrajectories
   * method can only handle a single trajectory at a time.
   */
  start_execution = ros::WallTime::now();
  ROS_INFO_STREAM(" Starting Execution of trajectories");
  for(unsigned int i = 0;i < ter_reqs.size(); i++)
  {
	  std::vector<TrajectoryExecutionRequest> tempRequest;
	  tempRequest.push_back(ter_reqs[i]);
	  ROS_INFO_STREAM("\t- "<<segment_names[i] <<" trajectory in progress");
//	  trajectory_execution_monitor_.executeTrajectories(tempRequest,
//														trajectories_finished_function_);
	  trajectory_execution_monitor_.executeTrajectories(tempRequest,callbacks[i]);
	  {
		  boost::unique_lock<boost::mutex> lock(execution_mutex_);
		  execution_completed_.wait(lock);

		  if(trajectories_succeeded_)
		  {
			  ROS_INFO_STREAM("\t- "<<segment_names[i] <<" trajectory completed");
		  }
		  else
		  {
			  ROS_ERROR_STREAM("\t- "<<segment_names[i] <<" trajectory interrupted, aborting pick move sequence");
			  break;
		  }
	  }

  }

  // moving arm to start of approach move and commanding a grasp release
  if(!trajectories_succeeded_ && performRecoveryMove)
  {
	  updateCurrentJointStateToLastTrajectoryPoint(last_trajectory_execution_data_vector_.back().recorded_trajectory_);

	  // moving arm back to start of pick move
	  ROS_WARN_STREAM(" Starting recovery move");
	  ROS_WARN_STREAM("\t- Moving arm to beginning of approach move trajectory");
	  if(!moveArm(group_name, gei.approach_trajectory_.points[0].positions))
	  {
		  ROS_ERROR_STREAM("\t-"<< "Move start pose failed, aborting recovery move");
		  return false;
	  }

	  // request gripper release command
	  object_manipulation_msgs::GraspHandPostureExecutionGoal graspGoal;
	  graspGoal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
	  ROS_WARN_STREAM("\t- Requesting grasp release");
	  grasp_exec_action_client_->sendGoal(graspGoal);
	  if(!grasp_exec_action_client_->waitForResult(ros::Duration(2.0f)))
	  {
		  ROS_ERROR_STREAM(" Release request timeout, aborting recovery move");
	  }

	  ROS_WARN_STREAM(" Completed recovery move");
	  return false;
  }

  ROS_INFO_STREAM("Finished executing all trajectories");
  execution_duration_ += (ros::WallTime::now()-start_execution);
  return trajectories_succeeded_ ;
}

bool RobotNavigator::attemptPlaceSequence(const std::string& group_name,
                          const object_manipulator::PlaceExecutionInfo& pei) {
  std::vector<TrajectoryExecutionRequest> ter_reqs;

  // moving arm to beginning of place move
  if(!moveArm(group_name,pei.descend_trajectory_.points[0].positions))
  {
    return false;
  }

  trajectories_succeeded_ = false;
  std::vector<std::string> segment_names;
  std::vector<boost::function<bool(TrajectoryExecutionDataVector)> > callbacks;

  // setting up descend move
  TrajectoryExecutionRequest arm_ter;
  arm_ter.group_name_ = group_name;
  arm_ter.controller_name_ = arm_controller_handler_->getControllerName();
  arm_ter.trajectory_ = pei.descend_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  performTrajectoryFiltering(group_name, arm_ter.trajectory_);
  arm_ter.failure_ok_ = false;
  arm_ter.monitor_overshoot_ = false;
  arm_ter.min_overshoot_time_ = ros::Duration(0.0f);
  arm_ter.max_overshoot_time_ = ros::Duration(5.0f);
  arm_ter.max_joint_distance_ = .1;
  arm_ter.failure_time_factor_ = 1000;
  arm_ter.callback_function_ = boost::bind(&RobotNavigator::detachCollisionObjectCallback, this, _1);
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("Descend");
  callbacks.push_back(trajectories_finished_function_);

  // setting up  gripper release
  TrajectoryExecutionRequest gripper_ter;
  gripper_ter.group_name_ = gripper_group_name_;
  gripper_ter.controller_name_ = grasp_action_service_;
  gripper_ter.trajectory_ = getGripperTrajectory(object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE);
  validateJointTrajectory(gripper_ter.trajectory_);
  gripper_ter.failure_ok_ = true;
  gripper_ter.test_for_close_enough_ = false;
  ter_reqs.push_back(gripper_ter);
  segment_names.push_back("Release");
  callbacks.push_back(grasp_action_finished_function_);

  // updating attached object marker operation
  if(hasMarker(MARKER_ATTACHED_OBJECT))
  {
	  visualization_msgs::Marker &m = getMarker(MARKER_ATTACHED_OBJECT);
	  m.action = visualization_msgs::Marker::DELETE;
	  addMarker(MARKER_ATTACHED_OBJECT,m);
  }

  //do the retreat
  arm_ter.trajectory_ = pei.retreat_trajectory_;
  validateJointTrajectory(arm_ter.trajectory_);
  performTrajectoryFiltering(group_name, arm_ter.trajectory_);
  arm_ter.callback_function_ = 0;
  ter_reqs.push_back(arm_ter);
  segment_names.push_back("Retreat");
  callbacks.push_back(trajectories_finished_function_);

  /*
   * executing all trajectories
   * currently this is the only way to execute multiple trajectories in sequence, since the executeTrajectories
   * method can only handle a single trajectory at a time.
   */
  ros::WallTime start_execution = ros::WallTime::now();
  ROS_INFO_STREAM( NODE_NAME <<": Starting Execution of trajectories");
  for(unsigned int i = 0;i < ter_reqs.size(); i++)
  {
	  std::vector<TrajectoryExecutionRequest> tempRequest;
	  tempRequest.push_back(ter_reqs[i]);

	  ROS_INFO_STREAM("\t"<< "- "<<segment_names[i] <<" trajectory in progress");
	  trajectory_execution_monitor_.executeTrajectories(tempRequest,callbacks[i]);

	  {
		  boost::unique_lock<boost::mutex> lock(execution_mutex_);
		  execution_completed_.wait(lock);

		  if(trajectories_succeeded_)
		  {
			  ROS_INFO_STREAM("\t"<< "- "<<segment_names[i] <<" trajectory completed");
		  }
		  else
		  {
			  ROS_INFO_STREAM("\t"<< "- "<<segment_names[i] <<" trajectory interrupted, aborting place sequence");
			  break;
		  }
	  }
  }
  ROS_INFO_STREAM(NODE_NAME <<"Finished executing all trajectories");

  execution_duration_ += (ros::WallTime::now()-start_execution);
  return trajectories_succeeded_;
}

trajectory_msgs::JointTrajectory RobotNavigator::getGripperTrajectory(int graspMove)
{
  trajectory_msgs::JointTrajectoryPoint jp;
  trajectory_msgs::JointTrajectory gt;

  gt.joint_names.push_back("j");
  gt.points.resize(1);
  gt.points[0].positions.resize(1, .1);
  gt.points[0].positions[0] = (double) graspMove;

  return gt;
}

bool RobotNavigator::attachCollisionObject(const std::string& group_name,
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
  state.updateKinematicStateWithLinkAt(gripper_link_name_, grasp_pose);

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

bool RobotNavigator::detachCollisionObject(const std::string& group_name,
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

void RobotNavigator::collisionObjToMarker(const arm_navigation_msgs::CollisionObject &obj,visualization_msgs::Marker &marker)
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

void RobotNavigator::startCycleTimer()
{
  execution_duration_ = ros::WallDuration(0.0);
  perception_duration_ = ros::WallDuration(0.0);
  planning_scene_duration_ = ros::WallDuration(0.0);
  motion_planning_duration_ = ros::WallDuration(0.0);
  trajectory_filtering_duration_ = ros::WallDuration(0.0);
  grasp_planning_duration_ = ros::WallDuration(0.0);
  cycle_start_time_ = ros::WallTime::now();
}

void RobotNavigator::printTiming()
{
    ros::WallDuration dur = ros::WallTime::now()-cycle_start_time_;
    ROS_INFO_STREAM("Cycle took " << dur.toSec() << " processing " << (dur-execution_duration_).toSec() << " execution " << execution_duration_.toSec());
    ROS_INFO_STREAM("Perception " << perception_duration_.toSec());
    ROS_INFO_STREAM("Planning scene " << planning_scene_duration_.toSec());
    ROS_INFO_STREAM("Planning time " << motion_planning_duration_.toSec());
    ROS_INFO_STREAM("Filtering time " << trajectory_filtering_duration_.toSec());
    ROS_INFO_STREAM("Grasp planning time " << grasp_planning_duration_.toSec());
  }

std::string RobotNavigator::makeCollisionObjectNameFromModelId(unsigned int model_id)
{
  std::stringstream object_id;
  object_id << "object_" << model_id;
  return object_id.str();
}

const arm_navigation_msgs::CollisionObject* RobotNavigator::getCollisionObject(unsigned int model_id)
{
    std::string name = makeCollisionObjectNameFromModelId(model_id);
    for(unsigned int i = 0; i < planning_scene_diff_.collision_objects.size(); i++) {
      if(planning_scene_diff_.collision_objects[i].id == name) return &(planning_scene_diff_.collision_objects[i]);
    }
    return NULL;
}

void RobotNavigator::run()
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
	    startCycleTimer();


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

		ROS_INFO_STREAM(NODE_NAME << ": Grasp Pickup stage started");
		if(!moveArmThroughPickSequence())
		{
		  ROS_WARN_STREAM(NODE_NAME << ": Grasp Pickup stage failed");
		  moveArmToSide();
		  continue;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": Grasp Pickup stage completed");
		}

		ROS_INFO_STREAM(NODE_NAME + ": Grasp Place stage started");
		if(!moveArmThroughPlaceSequence())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Grasp Place stage failed");
			moveArmToSide();
			continue;
		}
		else
		{
			ROS_INFO_STREAM(NODE_NAME << ": Grasp Place stage completed");
		}

		if(!moveArmToSide())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Side moved failed");
		}


	    printTiming();
	  }
}



