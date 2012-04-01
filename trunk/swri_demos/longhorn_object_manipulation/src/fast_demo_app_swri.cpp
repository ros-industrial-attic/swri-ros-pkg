#include <ros/ros.h>
#include <algorithm>

#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <object_manipulator/grasp_execution/grasp_tester_fast.h>
#include <object_manipulator/place_execution/place_tester_fast.h>
#include <trajectory_execution_monitor/trajectory_execution_monitor.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_execution_monitor/joint_state_recorder.h>
#include <trajectory_execution_monitor/follow_joint_trajectory_controller_handler.h>
#include <household_objects_database_msgs/GetModelMesh.h>
#include <household_objects_database_msgs/GetModelDescription.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <planning_environment/util/construct_object.h>
#include <longhorn_object_manipulation/grasp_posture_trajectory_controller_handler.h>

using namespace trajectory_execution_monitor;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

static inline std::string makeCollisionObjectNameFromModelId(unsigned int model_id) {
  std::stringstream object_id;
  object_id << "object_" << model_id;
  return object_id.str();
}

class FastDemoApp {

public:

  FastDemoApp() :
    cm_("robot_description"),
    current_robot_state_(NULL)
  {
    ros::NodeHandle nh;

    joint_state_recorder_.reset(new JointStateTrajectoryRecorder("/joint_states"));

    arm_controller_handler_.reset(new FollowJointTrajectoryControllerHandler("manipulator",
                                                                             "/joint_trajectory_action"));
    gripper_controller_handler_.reset(new GraspPostureTrajectoryControllerHandler("end_effector",
										  "/grasp_execution_action"));

    trajectory_execution_monitor_.addTrajectoryRecorder(joint_state_recorder_);
    trajectory_execution_monitor_.addTrajectoryControllerHandler(arm_controller_handler_);
    trajectory_execution_monitor_.addTrajectoryControllerHandler(gripper_controller_handler_);
    
    seg_srv_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);
    rec_srv_ = nh.serviceClient<tabletop_object_detector::TabletopObjectRecognition>("/tabletop_object_recognition", true);

    vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> ("swri", 128);
    vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> ("swri_array", 128);

    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    set_planning_scene_diff_client_ = 
      nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

    planning_service_client_ = nh.serviceClient<arm_navigation_msgs::GetMotionPlan>("/ompl_planning/plan_kinematic_path");
    //planning_service_client_ = nh.serviceClient<arm_navigation_msgs::GetMotionPlan>("/chomp_planner_longrange/plan_path");
    trajectory_filter_service_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("/trajectory_filter_server/filter_trajectory_with_constraints");
    //trajectory_filter_fast_service_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("/trajectory_filter_server_fast/filter_trajectory_with_constraints");

    object_database_model_mesh_client_ = nh.serviceClient<household_objects_database_msgs::GetModelMesh>("/objects_database_node/get_model_mesh", true);
    object_database_model_description_client_ = nh.serviceClient<household_objects_database_msgs::GetModelDescription>("/objects_database_node/get_model_description", true);

    object_database_grasp_client_ = nh.serviceClient<object_manipulation_msgs::GraspPlanning>("/plan_point_cluster_grasp", true);

    grasp_tester_ = new object_manipulator::GraspTesterFast(&cm_, "longhorn_manipulator_kinematics/IKFastKinematicsPlugin");
    place_tester_ = new object_manipulator::PlaceTesterFast(&cm_, "longhorn_manipulator_kinematics/IKFastKinematicsPlugin");

    trajectories_finished_function_ = boost::bind(&FastDemoApp::trajectoriesFinishedCallbackFunction, this, _1);

    attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);
  }
  
  void revertPlanningScene() {
    if(current_robot_state_ != NULL) {
      cm_.revertPlanningScene(current_robot_state_);
      current_robot_state_ = NULL;
    }
    last_mpr_id_ = 0;
    max_mpr_id_ = 0;
    max_trajectory_id_ = 0;
  }

  std::vector<std::string> getJointNames(const std::string& group) {
    if(cm_.getKinematicModel()->getModelGroup(group) == NULL) {
      std::vector<std::string> names;
      return names;
    }
    return cm_.getKinematicModel()->getModelGroup(group)->getJointModelNames();
  }

  void updateCurrentJointStateToLastTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj) 
  {
    if(traj.points.size() == 0) {
      ROS_ERROR_STREAM("No points in trajectory for setting current state");
      return;
    }
    std::map<std::string, double> joint_vals;
    for(unsigned int i = 0; i < traj.joint_names.size(); i++) {
      joint_vals[traj.joint_names[i]] = traj.points.back().positions[i];
    }
    current_robot_state_->setKinematicState(joint_vals);
  }
  
  bool getAndSetPlanningScene() {
    ros::WallTime start_time = ros::WallTime::now();
    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    
    revertPlanningScene();

    planning_scene_req.planning_scene_diff = planning_scene_diff_;

    ROS_DEBUG_STREAM("Getting and setting planning scene");

    if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res)) {
      ROS_WARN("Can't get planning scene");
      return false;
    }

    current_planning_scene_ = planning_scene_res.planning_scene;

    current_robot_state_ = cm_.setPlanningScene(current_planning_scene_);

    if(current_robot_state_ == NULL) {
      ROS_WARN("Problems setting local state");
      return false;
    }

    // Change time stamp to avoid saving sim time.
    current_planning_scene_.robot_state.joint_state.header.stamp = ros::Time(ros::WallTime::now().toSec());
    ROS_INFO_STREAM("Setting took " << (ros::WallTime::now()-start_time).toSec());
    planning_scene_duration_ += ros::WallTime::now()-start_time;
    return true;
  }
  
  bool trajectoriesFinishedCallbackFunction(TrajectoryExecutionDataVector tedv) {
    last_trajectory_execution_data_vector_ = tedv;
    trajectories_succeeded_ = (tedv.back().result_ == SUCCEEDED 
                               || tedv.back().result_ == HANDLER_REPORTS_FAILURE_BUT_OK 
                               || tedv.back().result_ == HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH); 
    ROS_INFO_STREAM("Trajectories " << tedv.size() << " ok " << trajectories_succeeded_);
    for(unsigned int i = 0; i < tedv.size(); i++) {
      ROS_INFO_STREAM("Recorded trajectory " << i << " has " << tedv[i].recorded_trajectory_.points.size());
    }
    execution_completed_.notify_all();
    return true;
  }

  bool moveArm(const std::string& group_name, 
               const std::vector<double>& joint_positions) {
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
    
    std::vector<arm_navigation_msgs::JointConstraint>& joint_constraints = 
      plan_req.motion_plan_request.goal_constraints.joint_constraints;

    joint_constraints.resize(joint_names.size());
    for(unsigned int i = 0; i < joint_names.size(); i++) {
      joint_constraints[i].joint_name = joint_names[i];
      joint_constraints[i].position = joint_positions[i];
      joint_constraints[i].tolerance_above = .01;
      joint_constraints[i].tolerance_below = .01;
    }

    if(!planning_service_client_.call(plan_req, plan_res)) {
      ROS_WARN_STREAM("Planner service call failed");
      return false;
    }
    
    if(plan_res.error_code.val != plan_res.error_code.SUCCESS) {
      ROS_WARN_STREAM("Planner failed");
      return false;
    }
    
    motion_planning_duration_ += ros::WallTime::now()-start_time;
    start_time = ros::WallTime::now();
    
    last_mpr_id_ = max_mpr_id_;
    max_mpr_id_++;
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;

    filter_req.trajectory = plan_res.trajectory.joint_trajectory;
    filter_req.start_state = plan_req.motion_plan_request.start_state;
    filter_req.group_name = group_name;
    filter_req.goal_constraints = plan_req.motion_plan_request.goal_constraints;
    filter_req.allowed_time = ros::Duration(2.0);

    if(!trajectory_filter_service_client_.call(filter_req, filter_res)) {
      ROS_WARN_STREAM("Filter service call failed");
      return false;
    }

    if(filter_res.error_code.val != filter_res.error_code.SUCCESS) {
      ROS_WARN_STREAM("Filter failed");
      return false;
    }
    trajectory_filtering_duration_ += ros::WallTime::now()-start_time;

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
    ROS_INFO_STREAM("Should be trying to move arm");
    trajectory_execution_monitor_.executeTrajectories(ter_reqs,
                                                      trajectories_finished_function_);
    boost::unique_lock<boost::mutex> lock(execution_mutex_);
    execution_completed_.wait(lock);
    execution_duration_ += (ros::WallTime::now()-start_execution);

    arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    if(trajectories_succeeded_) {
      error_code.val = error_code.SUCCESS;
    } else {
      error_code.val = error_code.PLANNING_FAILED;
    }

    return trajectories_succeeded_;
  }

  bool fastFilterTrajectory(const std::string& group_name,
                            trajectory_msgs::JointTrajectory& jt)
  {
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;
    
    std::map<std::string, double> jvals;
    for(unsigned int i = 0; i < jt.joint_names.size(); i++) {
      jvals[jt.joint_names[i]] = jt.points[0].positions[i];
    }
    planning_models::KinematicState state(*current_robot_state_);
    state.setKinematicState(jvals);

    filter_req.trajectory = jt;
    planning_environment::convertKinematicStateToRobotState(state,
                                                            ros::Time::now(),
                                                            cm_.getWorldFrameId(),
                                                            filter_req.start_state);
    filter_req.group_name = group_name;

    if(!trajectory_filter_service_client_.call(filter_req, filter_res)) {
      ROS_WARN_STREAM("Fast Filter service call failed");
      return false;
    }
    jt = filter_res.trajectory;
    return true;
  }

  bool moveArmToSide() {

    getAndSetPlanningScene();

    //name: ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
    //position: [-0.46665400266647339, 0.1069866344332695, 2.1171059608459473, -1.4666222333908081, -0.17949093878269196, -1.6554385423660278, -1.7431133985519409]

    //-3.0410828590393066, 0.10696959495544434, 2.117098093032837, -1.466621994972229, -0.17949093878269196, -1.6554234027862549, -1.7430833578109741    

    std::vector<double> joint_angles;
    joint_angles.push_back(-3.0410828590393066);
    joint_angles.push_back(0.1069866344332695);
    joint_angles.push_back(2.1171059608459473);
    joint_angles.push_back(-1.4666222333908081);
    joint_angles.push_back(-0.17949093878269196);
    joint_angles.push_back(-1.6554385423660278);
    joint_angles.push_back(-1.7431133985519409);
    return moveArm("manipulator",joint_angles);
  }

  void addDetectedTableToPlanningSceneDiff(const tabletop_object_detector::Table &table) {
    arm_navigation_msgs::CollisionObject table_object;
    table_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    table_object.shapes.resize(1);
    table_object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    table_object.shapes[0].dimensions.resize(3);
    table_object.shapes[0].dimensions[0] = fabs(table.x_max-table.x_min);
    table_object.shapes[0].dimensions[1] = fabs(table.y_max-table.y_min);
    table_object.shapes[0].dimensions[2] = 0.01;

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
    
    table_object.id = "table";

    ROS_INFO_STREAM("Table pose is " << table_object.poses[0].position.x << " "
                    << table_object.poses[0].position.y << " "
                    << table_object.poses[0].position.z);
 
    planning_scene_diff_.collision_objects.push_back(table_object);
    table_ = table_object;
  }

  void addDetectedObjectToPlanningSceneDiff(const household_objects_database_msgs::DatabaseModelPoseList& model) {
    arm_navigation_msgs::CollisionObject obj;
        
    obj.id = makeCollisionObjectNameFromModelId(model.model_list[0].model_id);

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
    ROS_INFO_STREAM("Pushing back " << obj.id << " frame " << obj.header.frame_id);
    planning_scene_diff_.collision_objects.push_back(obj);
  }

  bool segmentAndRecognize() {
    ros::WallTime start = ros::WallTime::now();
    planning_scene_diff_.collision_objects.clear();
    transformed_recognition_poses_.clear();
    tabletop_object_detector::TabletopSegmentation segmentation_srv;
    if (!seg_srv_.call(segmentation_srv))
    {
      ROS_ERROR("Call to segmentation service failed");
      return false;
    }
    ros::WallTime after_seg = ros::WallTime::now();
    ROS_INFO_STREAM("Seg took " << (after_seg-start).toSec());
    if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
    {
      ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
      return false;
    }
    addDetectedTableToPlanningSceneDiff(segmentation_srv.response.table);
    ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
    bool got_recognition = false;
    object_models_.clear();
    if(!segmentation_srv.response.clusters.empty()) {
      
      last_clusters_ = segmentation_srv.response.clusters;

      tabletop_object_detector::TabletopObjectRecognition recognition_srv;
      recognition_srv.request.table = segmentation_srv.response.table;
      recognition_srv.request.clusters = segmentation_srv.response.clusters;
      recognition_srv.request.num_models = 1;
      recognition_srv.request.perform_fit_merge = false;
      if (!rec_srv_.call(recognition_srv))
      {
        ROS_ERROR("Call to recognition service failed");
        //response.detection.result = response.detection.OTHER_ERROR;
      }
      ROS_INFO_STREAM("Recognition took " << (ros::WallTime::now()-after_seg));
      ROS_INFO_STREAM("Got " << recognition_srv.response.models.size() << " models");
      object_models_ = recognition_srv.response.models;
      for(unsigned int i = 0; i < 1; i++) { //recognition_srv.response.models.size(); i++) {
        got_recognition = true;
        addDetectedObjectToPlanningSceneDiff(recognition_srv.response.models[0]);
      }
      //response.detection.models = recognition_srv.response.models;
      //response.detection.cluster_model_indices = recognition_srv.response.cluster_model_indices;
    }
    perception_duration_ += ros::WallTime::now()-start;
    return got_recognition;
  }
  
  bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
                               arm_navigation_msgs::CollisionObject& obj,
                               const geometry_msgs::PoseStamped& pose)
  {
    household_objects_database_msgs::GetModelMesh::Request req;
    household_objects_database_msgs::GetModelMesh::Response res;
    
    req.model_id = model_pose.model_id;
    if(!object_database_model_mesh_client_.call(req, res)) {
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

    transformed_recognition_poses_[obj.id] = pose;

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

  bool selectGraspableObject(const std::string& arm_name,
                             object_manipulation_msgs::PickupGoal& pickup_goal,
                             std::vector<object_manipulation_msgs::Grasp>& grasps)
  {
    if(object_models_.size() == 0) return false;
    if(object_models_[0].model_list.size() == 0) return false;
    
    household_objects_database_msgs::DatabaseModelPose& dmp = object_models_[0].model_list[0];

    pickup_goal.arm_name = arm_name;
    pickup_goal.collision_object_name = makeCollisionObjectNameFromModelId(dmp.model_id);
    pickup_goal.lift.direction.header.frame_id = cm_.getWorldFrameId();
    pickup_goal.lift.direction.vector.z = 1.0;
    pickup_goal.lift.desired_distance = .1;
    pickup_goal.target.reference_frame_id = pickup_goal.collision_object_name;
    pickup_goal.target.cluster = last_clusters_[0];
    pickup_goal.allow_gripper_support_collision = true;
    pickup_goal.collision_support_surface_name = "table";
    

    household_objects_database_msgs::DatabaseModelPose dmp_copy = dmp;
    dmp_copy.pose = transformed_recognition_poses_[pickup_goal.collision_object_name];
    pickup_goal.target.potential_models.push_back(dmp_copy);
    return getObjectGrasps(arm_name,
                           dmp,
			   last_clusters_[0],
			   grasps);
  }

  bool putDownSomething(const std::string& arm_name) {
    
    getAndSetPlanningScene();

    ros::WallTime start_time = ros::WallTime::now();

    if(!object_in_hand_map_[arm_name]) return false;

    object_manipulation_msgs::PlaceGoal place_goal;
    place_goal.arm_name = arm_name;
    place_goal.grasp = current_grasp_map_[arm_name];
    place_goal.desired_retreat_distance = .1;
    place_goal.min_retreat_distance = .1;
    place_goal.approach.desired_distance = .1;
    place_goal.approach.min_distance = .1;
    place_goal.approach.direction.header.frame_id = cm_.getWorldFrameId();
    place_goal.approach.direction.vector.x = 0.0;
    place_goal.approach.direction.vector.y = 0.0;
    place_goal.approach.direction.vector.z = -1.0;
    place_goal.collision_object_name = "attached_"+current_grasped_object_name_[arm_name];
    place_goal.allow_gripper_support_collision = true;
    place_goal.collision_support_surface_name = "table";
    place_goal.allow_gripper_support_collision = true;
    place_goal.place_padding = .02;

    geometry_msgs::PoseStamped table_pose;
    table_pose.pose = table_.poses[0];
    table_pose.header.frame_id = table_.header.frame_id;

    double l = table_.shapes[0].dimensions[0]-.2;
    double w = table_.shapes[0].dimensions[1]-.2;
    double d = table_.shapes[0].dimensions[2];

    double spacing = .1;
    
    unsigned int lnum = ceil(l/spacing);
    unsigned int wnum = ceil(w/spacing);

    std::vector<double> angles;
    angles.push_back(0);
    angles.push_back(M_PI/4.0);
    angles.push_back(-M_PI/4.0);
    angles.push_back(M_PI/2.0);
    angles.push_back(-M_PI/2.0);

    unsigned int total_place_locations = lnum*wnum*angles.size();

    std::vector<unsigned int> random_numbers(total_place_locations);
    for(unsigned int i = 0; i < total_place_locations; i++) {
      random_numbers[i] = i;
    }
    random_shuffle(random_numbers.begin(), random_numbers.end());

    std::vector<geometry_msgs::PoseStamped> place_locations(total_place_locations);
    //TODO - make sure that poses are actually on the table
    unsigned int cur_ind = 0;
    for(unsigned int i = 0; i < lnum; i++) {
      for(unsigned int j = 0; j < wnum; j++) { 
        for(unsigned int k = 0; k < angles.size(); k++, cur_ind++) {
          geometry_msgs::PoseStamped place_pose = table_pose;
          place_pose.pose.position.x += -(l/2.0)+((i*1.0)*spacing);
          place_pose.pose.position.y += -(w/2.0)+((j*1.0)*spacing);
          place_pose.pose.position.z += (d/2.0);
          tf::Transform rot(tf::Quaternion(tf::Vector3(0.0,0.0,1.0), angles[k]), tf::Vector3(0.0,0.0,0.0));
          tf::Transform trans;
          tf::poseMsgToTF(place_pose.pose, trans);
          trans = trans*rot;
          tf::poseTFToMsg(trans, place_pose.pose);
          ROS_INFO_STREAM("Place location " << i << " " << j << " " 
                           << place_pose.pose.position.x << " " 
                           << place_pose.pose.position.y << " " 
                           << place_pose.pose.position.z);
          place_locations[random_numbers[cur_ind]] = place_pose;
        }
      }
    }

    std::vector<object_manipulator::PlaceExecutionInfo> place_execution_info;
    {
      planning_models::KinematicState state(*current_robot_state_);
      place_tester_->setPlanningSceneState(&state);
      place_tester_->testPlaces(place_goal,
                                place_locations,
                                place_execution_info,
                                true);
    }

    grasp_planning_duration_ += ros::WallTime::now()-start_time;
    
    for(unsigned int i = 0; i < place_execution_info.size(); i++) {
      if(place_execution_info[i].result_.result_code == object_manipulation_msgs::PlaceLocationResult::SUCCESS) {
        current_place_location_ = place_locations[i];
        bool placed = attemptPlaceSequence(arm_name,
                                           place_execution_info[i]);
        if(!placed) {
          ROS_WARN_STREAM("Place failed");
        } else {
          return true;
        }
      }
    }
    return false;
  }

  bool pickUpSomething(const std::string& arm_name) {

    getAndSetPlanningScene();

    ros::WallTime start_time = ros::WallTime::now();

    object_manipulation_msgs::PickupGoal pickup_goal;
    std::vector<object_manipulation_msgs::Grasp> grasps;
    if(!selectGraspableObject(arm_name,
                              pickup_goal,
                              grasps)) {
      return false;
    }
    std::vector<object_manipulator::GraspExecutionInfo> grasp_execution_info;
    
    {
      planning_models::KinematicState state(*current_robot_state_);
      grasp_tester_->setPlanningSceneState(&state);
      ROS_INFO_STREAM("Pickup goal arm name is " << pickup_goal.arm_name);
      grasp_tester_->testGrasps(pickup_goal,
                                grasps,
                                grasp_execution_info,
                                true);
    }

    grasp_planning_duration_ += ros::WallTime::now()-start_time;

    for(unsigned int i = 0; i < grasp_execution_info.size(); i++) {
      if(grasp_execution_info[i].result_.result_code == object_manipulation_msgs::GraspResult::SUCCESS) {
        current_grasped_object_name_[arm_name] = pickup_goal.collision_object_name;
        current_grasp_map_[arm_name] = grasps[i];
        bool grasped =  attemptGraspSequence(arm_name, grasp_execution_info[i]);
        if(!grasped) {
          ROS_WARN_STREAM("Grasp failed");
          current_grasped_object_name_.erase(arm_name);
          current_grasp_map_.erase(arm_name);
	  return false;
        } else {
          return true;
        }
      }
    }
    return false;
  }

  void attachCollisionObjectCallback(const std::string& group_name) {
    ROS_INFO_STREAM("In attach callback");
    attachCollisionObject(group_name,
                          current_grasped_object_name_[group_name],
                          current_grasp_map_[group_name]);
    object_in_hand_map_[group_name] = true;
  }

  void detachCollisionObjectCallback(const std::string& group_name) {
    ROS_INFO_STREAM("In detach callback");
    detachCollisionObject(group_name,
                          current_place_location_,
                          current_grasp_map_[group_name]);
    object_in_hand_map_[group_name] = false;
    current_grasped_object_name_.erase(group_name);
    current_grasp_map_.erase(group_name);
  }

  bool attemptGraspSequence(const std::string& group_name,
                            const object_manipulator::GraspExecutionInfo& gei) {
    //first open the gripper
    std::vector<TrajectoryExecutionRequest> ter_reqs;
    TrajectoryExecutionRequest gripper_ter;
    gripper_ter.group_name_ = "end_effector";
    gripper_ter.controller_name_ = "/grasp_execution_action";
    gripper_ter.trajectory_ = getGripperTrajectory(group_name, true);
    gripper_ter.failure_ok_ = true;
    gripper_ter.test_for_close_enough_ = false;
    ter_reqs.push_back(gripper_ter);

    ros::WallTime start_execution = ros::WallTime::now();
    trajectory_execution_monitor_.executeTrajectories(ter_reqs,
                                                      trajectories_finished_function_);
    {
      boost::unique_lock<boost::mutex> lock(execution_mutex_);
      execution_completed_.wait(lock);
    }
    execution_duration_ += (ros::WallTime::now()-start_execution);
    ROS_INFO_STREAM("Opened gripper");
    ter_reqs.clear();

    updateCurrentJointStateToLastTrajectoryPoint(last_trajectory_execution_data_vector_.back().recorded_trajectory_);

    if(!moveArm(group_name,
                gei.approach_trajectory_.points[0].positions)) {
      return false;
    }

    trajectories_succeeded_ = false;

    //now do approach
    TrajectoryExecutionRequest arm_ter;
    arm_ter.group_name_ = group_name;
    arm_ter.controller_name_ = arm_controller_handler_->getControllerName();
    arm_ter.trajectory_ = gei.approach_trajectory_;
    fastFilterTrajectory(group_name, arm_ter.trajectory_);
    arm_ter.test_for_close_enough_ = false;
    arm_ter.max_joint_distance_ = .05;
    arm_ter.failure_time_factor_ = 1000;

    arm_ter.callback_function_ = boost::bind(&FastDemoApp::attachCollisionObjectCallback, this, _1);
    ter_reqs.push_back(arm_ter);

    //now close the gripper
    gripper_ter.trajectory_ = getGripperTrajectory(group_name, false);
    ter_reqs.push_back(gripper_ter);

    //and do the lift
    arm_ter.trajectory_ = gei.lift_trajectory_;
    fastFilterTrajectory(group_name, arm_ter.trajectory_);
    arm_ter.callback_function_ = 0;
    ter_reqs.push_back(arm_ter);

    start_execution = ros::WallTime::now();
    trajectory_execution_monitor_.executeTrajectories(ter_reqs,
                                                      trajectories_finished_function_);
    boost::unique_lock<boost::mutex> lock(execution_mutex_);
    execution_completed_.wait(lock);

    execution_duration_ += (ros::WallTime::now()-start_execution);
    std::vector<std::string> segment_names;
    segment_names.push_back("approach");
    segment_names.push_back("close_gripper");
    segment_names.push_back("lift");
    
    return trajectories_succeeded_;
  }

  bool attemptPlaceSequence(const std::string& group_name,
                            const object_manipulator::PlaceExecutionInfo& pei) {
    std::vector<TrajectoryExecutionRequest> ter_reqs;

    if(!moveArm(group_name,
                pei.descend_trajectory_.points[0].positions)) {
      return false;
    }

    trajectories_succeeded_ = false;

    //now do descend
    TrajectoryExecutionRequest arm_ter;
    arm_ter.group_name_ = group_name;
    arm_ter.controller_name_ = arm_controller_handler_->getControllerName();
    arm_ter.trajectory_ = pei.descend_trajectory_;
    fastFilterTrajectory(group_name, arm_ter.trajectory_);
    arm_ter.test_for_close_enough_ = false;
    arm_ter.max_joint_distance_ = .05;
    arm_ter.failure_time_factor_ = 1000;
    arm_ter.callback_function_ = boost::bind(&FastDemoApp::detachCollisionObjectCallback, this, _1);
    ter_reqs.push_back(arm_ter);

    //open gripper
    TrajectoryExecutionRequest gripper_ter;
    gripper_ter.group_name_ = "end_effector";
    gripper_ter.controller_name_ = "/grasp_execution_action";
    gripper_ter.trajectory_ = getGripperTrajectory(group_name, true);
    gripper_ter.failure_ok_ = true;
    gripper_ter.test_for_close_enough_ = false;
    ter_reqs.push_back(gripper_ter);

    //do the retreat
    arm_ter.trajectory_ = pei.retreat_trajectory_;
    fastFilterTrajectory(group_name, arm_ter.trajectory_);
    arm_ter.callback_function_ = 0;
    ter_reqs.push_back(arm_ter);

    ros::WallTime start_execution = ros::WallTime::now();
    trajectory_execution_monitor_.executeTrajectories(ter_reqs,
                                                      trajectories_finished_function_);
    boost::unique_lock<boost::mutex> lock(execution_mutex_);
    execution_completed_.wait(lock);

    execution_duration_ += (ros::WallTime::now()-start_execution);
    std::vector<std::string> segment_names;
    segment_names.push_back("descend");
    segment_names.push_back("open_gripper");
    segment_names.push_back("retreat");
    return trajectories_succeeded_;
  }

  bool getObjectGrasps(const std::string& arm_name,
                       const household_objects_database_msgs::DatabaseModelPose& dmp,
                       const sensor_msgs::PointCloud& cloud,
                       std::vector<object_manipulation_msgs::Grasp>& grasps)
  {
    household_objects_database_msgs::GetModelDescription::Request des_req;
    household_objects_database_msgs::GetModelDescription::Response des_res;
    
    des_req.model_id = dmp.model_id;
    if(!object_database_model_description_client_.call(des_req, des_res)) {
      ROS_WARN_STREAM("Call to objects database for getModelDescription failed");
      return false;
    }
    if(des_res.return_code.code != des_res.return_code.SUCCESS) {
      ROS_WARN_STREAM("Object database gave non-success code " << des_res.return_code.code << " for model id " << des_req.model_id);
      return false;
    }
    household_objects_database_msgs::DatabaseModelPose dmp_copy = dmp;
    dmp_copy.pose = geometry_msgs::PoseStamped();
    dmp_copy.pose.pose.orientation.w = 1.0;
    dmp_copy.pose.header.frame_id = des_res.name;
    dmp_copy.pose.header.stamp = ros::Time::now();


    object_manipulation_msgs::GraspPlanning::Request request;
    object_manipulation_msgs::GraspPlanning::Response response;
      
    request.arm_name = arm_name;
    request.target.potential_models.push_back(dmp_copy);
    //request.target.reference_frame_id = des_res.name;
    request.target.reference_frame_id = cloud.header.frame_id;
    request.target.cluster = cloud;
      
    if(!object_database_grasp_client_.call(request, response)) {
      ROS_WARN_STREAM("Call to objects database for GraspPlanning failed");
      return false;
    }
    if(response.error_code.value != response.error_code.SUCCESS) {
      ROS_WARN_STREAM("Object database gave non-success code " << response.error_code.value);
      return false;
    }
    
    if(response.grasps.size() == 0) {
      ROS_WARN_STREAM("No grasps returned in response");
      return false;
    }  
    if(response.grasps.size() > 0) {
      ROS_INFO_STREAM("Grasp is " << response.grasps[0].grasp_pose.position.x << " "
		      << response.grasps[0].grasp_pose.position.y << " "
		      << response.grasps[0].grasp_pose.position.z);
    }

    ROS_INFO_STREAM("Cloud header " << cloud.header.frame_id);
    ROS_INFO_STREAM("Recognition pose frame " << dmp.pose.header.frame_id);

    tf::Transform first_grasp_in_world_tf;
    tf::poseMsgToTF(response.grasps[0].grasp_pose, first_grasp_in_world_tf);

    planning_models::KinematicState state(*current_robot_state_);
    state.updateKinematicStateWithLinkAt("palm", first_grasp_in_world_tf);

    std_msgs::ColorRGBA col_pregrasp;
    col_pregrasp.r = 0.0;
    col_pregrasp.g = 1.0;
    col_pregrasp.b = 1.0;
    col_pregrasp.a = 1.0;
    visualization_msgs::MarkerArray arr;
    
    std::vector<std::string> links = cm_.getKinematicModel()->getModelGroup("end_effector")->getGroupLinkNames();

    cm_.getRobotMarkersGivenState(state, arr, col_pregrasp,
				  "first_grasp",
				  ros::Duration(0.0), 
				  &links);
    vis_marker_array_publisher_.publish(arr);
    
    //TODO - actually deal with the different cases here, especially for the cluster planner
    grasps = response.grasps;
    if(request.target.reference_frame_id != des_res.name) {
      if(request.target.reference_frame_id != dmp.pose.header.frame_id) {
        ROS_WARN_STREAM("Cluster does not match recognition");
      } else {
        tf::Transform object_in_world_tf;
        tf::poseMsgToTF(dmp.pose.pose, object_in_world_tf);

        tf::Transform object_in_world_inverse_tf = object_in_world_tf.inverse();
        //poses are positions of the wrist link in terms of the world
        //we need to get them in terms of the object
        for(unsigned int i = 0; i < grasps.size(); i++) {
          tf::Transform grasp_in_world_tf;
          tf::poseMsgToTF(grasps[i].grasp_pose, grasp_in_world_tf);
          tf::poseTFToMsg(object_in_world_inverse_tf*grasp_in_world_tf, grasps[i].grasp_pose);
        }
      }
    }
    return true;
  }  
  
  trajectory_msgs::JointTrajectory getGripperTrajectory(const std::string& arm_name,
                                                        bool open) {
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

  bool attachCollisionObject(const std::string& group_name,
                             const std::string& collision_object_name,
                             const object_manipulation_msgs::Grasp& grasp) {
    arm_navigation_msgs::AttachedCollisionObject att;
    bool found = false;
    for(std::vector<arm_navigation_msgs::CollisionObject>::iterator it = planning_scene_diff_.collision_objects.begin();
        it != planning_scene_diff_.collision_objects.end(); 
        it++) {
      if((*it).id == collision_object_name) {
        found = true;
        att.object = (*it);
        planning_scene_diff_.collision_objects.erase(it);
        break;
      }
    }
    if(!found) {
      ROS_ERROR_STREAM("No object with id " << collision_object_name);
      return false;
    }
    att.link_name = "palm";
    att.touch_links.push_back("end_effector");
    
    planning_models::KinematicState state(*current_robot_state_);
    tf::Transform t;
    tf::poseMsgToTF(grasp.grasp_pose, t);

    std::map<std::string, double> grasp_vals;
    for(unsigned int i = 0; i < grasp.grasp_posture.name.size(); i ++) {
       grasp_vals[grasp.grasp_posture.name[i]] = grasp.grasp_posture.position[i];
    }
    state.setKinematicState(grasp_vals);

    tf::Transform obj_pose;
    tf::poseMsgToTF(transformed_recognition_poses_[collision_object_name].pose, obj_pose);
    
    //assumes that the grasp is in the object frame
    tf::Transform grasp_pose = obj_pose*t;

    // //need to do this second, otherwise it gets wiped out
    state.updateKinematicStateWithLinkAt("palm",
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

    ROS_INFO_STREAM("Res pose is " 
                    << ps.pose.position.x << " " 
                    << ps.pose.position.y << " " 
                    << ps.pose.position.z); 
    
    ROS_INFO_STREAM("Trying to add " << att.object.id << " mode " << att.object.operation.operation);
    planning_scene_diff_.attached_collision_objects.push_back(att);

    attached_object_publisher_.publish(att);
    return true;
  }

  bool detachCollisionObject(const std::string& group_name,
                             const geometry_msgs::PoseStamped& place_pose,
                             const object_manipulation_msgs::Grasp& grasp) {

    ROS_INFO_STREAM("Place pose is " << place_pose.header.frame_id);

    planning_models::KinematicState state(*current_robot_state_);
    tf::Transform place_pose_tf;
    tf::poseMsgToTF(place_pose.pose, place_pose_tf);

    tf::Transform grasp_trans;
    tf::poseMsgToTF(grasp.grasp_pose, grasp_trans);

    place_pose_tf = place_pose_tf*grasp_trans;
    
    state.updateKinematicStateWithLinkAt("palm",
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

  void startCycleTimer() {
    execution_duration_ = ros::WallDuration(0.0);
    perception_duration_ = ros::WallDuration(0.0);
    planning_scene_duration_ = ros::WallDuration(0.0);
    motion_planning_duration_ = ros::WallDuration(0.0);
    trajectory_filtering_duration_ = ros::WallDuration(0.0);
    grasp_planning_duration_ = ros::WallDuration(0.0);
    cycle_start_time_ = ros::WallTime::now();
  }

  void printTiming() {
    ros::WallDuration dur = ros::WallTime::now()-cycle_start_time_;
    ROS_INFO_STREAM("Cycle took " << dur.toSec() << " processing " << (dur-execution_duration_).toSec() << " execution " << execution_duration_.toSec());
    ROS_INFO_STREAM("Perception " << perception_duration_.toSec());
    ROS_INFO_STREAM("Planning scene " << planning_scene_duration_.toSec());
    ROS_INFO_STREAM("Planning time " << motion_planning_duration_.toSec());
    ROS_INFO_STREAM("Filtering time " << trajectory_filtering_duration_.toSec());
    ROS_INFO_STREAM("Grasp planning time " << grasp_planning_duration_.toSec());
  }
  
protected:

  const arm_navigation_msgs::CollisionObject* getCollisionObject(unsigned int model_id) {
    std::string name = makeCollisionObjectNameFromModelId(model_id);
    for(unsigned int i = 0; i < planning_scene_diff_.collision_objects.size(); i++) {
      if(planning_scene_diff_.collision_objects[i].id == name) return &(planning_scene_diff_.collision_objects[i]);
    }
    return NULL;
  }

  planning_environment::CollisionModels cm_;
  unsigned int current_planning_scene_id_;
  unsigned int last_mpr_id_;
  unsigned int max_mpr_id_;
  unsigned int max_trajectory_id_;

  TrajectoryExecutionDataVector last_trajectory_execution_data_vector_;

  ros::ServiceClient seg_srv_;
  ros::ServiceClient rec_srv_;

  ros::ServiceClient planning_service_client_;
  ros::ServiceClient trajectory_filter_service_client_;
  //ros::ServiceClient trajectory_filter_fast_service_client_;

  ros::ServiceClient object_database_model_mesh_client_;
  ros::ServiceClient object_database_grasp_client_;
  ros::ServiceClient object_database_model_description_client_;

  ros::ServiceClient set_planning_scene_diff_client_;

  boost::shared_ptr<TrajectoryRecorder> joint_state_recorder_;

  boost::shared_ptr<TrajectoryControllerHandler> arm_controller_handler_;
  boost::shared_ptr<TrajectoryControllerHandler> gripper_controller_handler_;

  trajectory_execution_monitor::TrajectoryExecutionMonitor trajectory_execution_monitor_;
  boost::function<bool(TrajectoryExecutionDataVector)> trajectories_finished_function_;

  object_manipulator::GraspTesterFast* grasp_tester_;
  object_manipulator::PlaceTesterFast* place_tester_;

  arm_navigation_msgs::PlanningScene current_planning_scene_;
  arm_navigation_msgs::PlanningScene planning_scene_diff_;
  planning_models::KinematicState* current_robot_state_;

  boost::condition_variable execution_completed_;
  boost::mutex execution_mutex_;
  bool trajectories_succeeded_;

  std::map<std::string, geometry_msgs::PoseStamped> transformed_recognition_poses_;
  std::vector<household_objects_database_msgs::DatabaseModelPoseList> object_models_;
  std::vector<sensor_msgs::PointCloud> last_clusters_;
  arm_navigation_msgs::CollisionObject table_;

  std::map<std::string, bool> object_in_hand_map_;
  std::map<std::string, std::string> current_grasped_object_name_;
  std::map<std::string, object_manipulation_msgs::Grasp> current_grasp_map_;
  
  geometry_msgs::PoseStamped current_place_location_;

  ros::WallTime cycle_start_time_;
  ros::WallDuration execution_duration_;
  ros::WallDuration perception_duration_;
  ros::WallDuration planning_scene_duration_;
  ros::WallDuration motion_planning_duration_;
  ros::WallDuration trajectory_filtering_duration_;
  ros::WallDuration grasp_planning_duration_;

  ros::Publisher attached_object_publisher_;

  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;

};

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "fast_demo_app", ros::init_options::NoSigintHandler);

  ros::AsyncSpinner spinner(4); 
  spinner.start();
  
  srand(time(NULL));
  
  ros::NodeHandle nh;

  FastDemoApp fda;

  fda.moveArmToSide();

  while(1) {
    fda.startCycleTimer();
    if(!fda.segmentAndRecognize()) {
      ROS_WARN_STREAM("Segment and recognized failed");
      continue;
    }
    if(!fda.pickUpSomething("manipulator")) {
      ROS_WARN_STREAM("Pick up failed");
      break;
    }
    if(!fda.putDownSomething("manipulator")) {
      ROS_WARN_STREAM("Put down failed");
    }
    if(!fda.moveArmToSide()) {
      ROS_WARN_STREAM("Final side moved failed");
      break;
    }
    fda.printTiming();
  }

  ros::shutdown();
}
