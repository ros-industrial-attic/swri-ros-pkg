/*
 * RobotNavigator.h
 *
 *  Created on: Oct 18, 2012
 *      Author: jnicho
 *  Description:
 *  	This class provides the basic functionality for robot arm navigation tasks.  These task include calling various
 *  	services through service clients for each step in the navigation sequence such as segmentation, recognition, grasp
 *  	planning, path planning, trajectory filtering, and grasp and trajectory executions.
 */

#ifndef ROBOTNAVIGATOR_H_
#define ROBOTNAVIGATOR_H_

#include <ros/ros.h>
#include <algorithm>
#include <trajectory_execution_monitor/joint_state_recorder.h>
#include <trajectory_execution_monitor/follow_joint_trajectory_controller_handler.h>
#include <trajectory_execution_monitor/trajectory_execution_monitor.h>
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
#include <actionlib/client/simple_action_client.h>
#include <household_objects_database_msgs/GetModelMesh.h>
#include <household_objects_database_msgs/GetModelDescription.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <planning_environment/util/construct_object.h>
#include <object_manipulation_tools/controller_utils/GraspPoseControllerHandler.h>
#include <object_manipulation_tools/manipulation_utils/PlaceSequenceValidator.h>
#include <object_manipulation_tools/manipulation_utils/GraspSequenceValidator.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction>  GraspActionServerClient;
//typedef boost::shared_ptr<object_manipulator::GraspTesterFast> GraspTesterPtr;
typedef boost::shared_ptr<GraspSequenceValidator> GraspTesterPtr;
typedef boost::shared_ptr<object_manipulator::GraspTesterFast> GraspTesterFastPtr;
typedef boost::shared_ptr<PlaceSequenceValidator> PlaceSequencePtr;

using namespace trajectory_execution_monitor;

// ros param default values
namespace RobotNavigatorParameters
{
	static const std::string DEFAULT_ARM_GROUP = "sia20d_arm";
	static const std::string DEFAULT_GRIPPER_GROUP = "end_effector";
	static const std::string DEFAULT_WRIST_LINK = "link_t";
	static const std::string DEFAULT_GRIPPER_LINK = "palm";
	static const std::string DEFAULT_GRASP_ACTION_SERVICE = "/grasp_execution_action";
	static const std::string DEFAULT_TRAJECTORY_ACTION_SERVICE = "/joint_trajectory_action";
	static const std::string DEFAULT_PATH_PLANNER_SERVICE = "/ompl_planning/plan_kinematic_path";
	static const std::string DEFAULT_SEGMENTATION_SERVICE = "/tabletop_segmentation";
	static const std::string DEFAULT_RECOGNITION_SERVICE = "/tabletop_object_recognition";
	static const std::string DEFAULT_TRAJECTORY_FILTER_SERVICE = "/trajectory_filter_server/filter_trajectory_with_constraints";
	static const std::string DEFAULT_GRASP_PLANNING_SERVICE = "/plan_point_cluster_grasp";
	static const std::string DEFAULT_MESH_DATABASE_SERVICE = "/objects_database_node/get_model_mesh";
	static const std::string DEFAULT_MODEL_DATABASE_SERVICE = "/objects_database_node/get_model_description";
	static const std::string DEFAULT_PLANNING_SCENE_SERVICE = "/environment_server/set_planning_scene_diff";
	static const std::string DEFAULT_IK_PLUGING = "SIA20D_Mesh_manipulator_kinematics/IKFastKinematicsPlugin";
	static const std::string DEFAULT_JOINT_STATES_TOPIC = "/joint_states";
	static const double DF_PICK_APPROACH_DISTANCE = 0.1f;
	static const double DF_PLACE_APPROACH_DISTANCE = 0.1f;
	static const double DF_PLACE_RETREAT_DISTANCE = 0.1f;

	// ros param names
	static const std::string PARAM_NAME_ARM_GROUP = "arm_group";
	static const std::string PARAM_NAME_GRIPPER_GROUP = "gripper_group";
	static const std::string PARAM_NAME_WRIST_LINK = "wrist_link";
	static const std::string PARAM_NAME_GRIPPER_LINK = "gripper_link";
	static const std::string PARAM_NAME_GRASP_ACTION_SERVICE = "grasp_action_service_name";
	static const std::string PARAM_NAME_TRAJECTORY_ACTION_SERVICE = "joint_trajectory_service_name";
	static const std::string PARAM_NAME_PATH_PLANNER_SERVICE = "planner_service_name";
	static const std::string PARAM_NAME_TRAJECTORY_FILTER_SERVICE = "trajectory_filter_service_name";
	static const std::string PARAM_NAME_SEGMENTATION_SERVICE = "segmentation_service_name";
	static const std::string PARAM_NAME_RECOGNITION_SERVICE = "recognition_service_name";
	static const std::string PARAM_NAME_GRASP_PLANNING_SERVICE = "grasp_planning_service_name";
	static const std::string PARAM_NAME_MESH_DATABASE_SERVICE = "mesh_database_service_name";
	static const std::string PARAM_NAME_MODEL_DATABASE_SERVICE = "model_database_service_name";
	static const std::string PARAM_NAME_PLANNING_SCENE_SERVICE = "planning_scene_service_name";
	static const std::string PARAM_NAME_IK_PLUGING = "arm_inverse_kinematics_plugin";
	static const std::string PARAM_NAME_JOINT_STATES_TOPIC = "joint_state_topic";
	static const std::string PARAM_PICK_APPROACH_DISTANCE = "/pick_approach_distance";
	static const std::string PARAM_PLACE_APPROACH_DISTANCE = "/place_approach_distance";
	static const std::string PARAM_PLACE_RETREAT_DISTANCE = "/place_retreat_distance";

}


class RobotNavigator
{
public:

	typedef boost::shared_ptr<RobotNavigator> Ptr;

public:
	RobotNavigator();
	virtual ~RobotNavigator();

	// demo start
	virtual void run();

	static std::string NODE_NAME;
	static std::string NAVIGATOR_NAMESPACE;
	static std::string MARKER_ARM_LINK;
	static std::string MARKER_ATTACHED_OBJECT;
	static std::string VISUALIZATION_TOPIC;

	// others
	static const tf::Transform PLACE_RECTIFICATION_TF; // used to orient the tcp's z vector in the normal direction for all place moves

protected:
	// setup
	virtual void setup();

	// ros parameters
	virtual void fetchParameters(std::string nameSpace = "");

	/* Service methods
	 * these methods perform the following:
	 * 	-  Build request objects,
	 * 	-  Check results
	 * 	-  Store results needed for subsequent steps in the manipulation
	 * 	-  Perform additional post processing stuff such as updating the planning scene or publish data
	 */
	virtual bool performSegmentation();
	virtual bool performRecognition();
	virtual bool performPickGraspPlanning();
	virtual bool performPlaceGraspPlanning();
	virtual bool performGraspPlanning();
	virtual bool performTrajectoryFiltering(const std::string& group_name,trajectory_msgs::JointTrajectory& jt);

	// goal position: this should be implemented by each specific specialization of the class
	virtual bool createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses) = 0;

	// database comm
	bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
			arm_navigation_msgs::CollisionObject& obj,const geometry_msgs::PoseStamped& pose);

	// move arm methods
	virtual bool moveArmToSide(); // it is preferable to implement this in order to tell the robot arm where "side" is
	virtual bool moveArm(const std::string& group_name,const std::vector<double>& joint_positions);
	virtual bool moveArmThroughPickSequence();
	virtual bool moveArmThroughPlaceSequence();

	// callbacks
	virtual void callbackPublishMarkers(const ros::TimerEvent &evnt);
	virtual bool trajectoryFinishedCallback(bool storeLastTraj,TrajectoryExecutionDataVector tedv);

	// planning scene
	void attachCollisionObjectCallback(const std::string& group_name);
	void detachCollisionObjectCallback(const std::string& group_name);
	bool attachCollisionObject(const std::string& group_name,const std::string& collision_object_name,
			const object_manipulation_msgs::Grasp& grasp);
	bool detachCollisionObject(const std::string& group_name,const geometry_msgs::PoseStamped& place_pose,
			const object_manipulation_msgs::Grasp& grasp);
	void revertPlanningScene();
	void updateCurrentJointStateToLastTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj);

	// adds or removes objects into planning scene through a service call, service returns a copy of the scene in its current state;
	bool updateChangesToPlanningScene();

	// add objects to local copy of the planning scene, however the planning scene service needs to be called in order to push
	// any changes into the actual planning scene
	void addDetectedTableToLocalPlanningScene(const tabletop_object_detector::Table &table);
	void addDetectedObjectToLocalPlanningScene(arm_navigation_msgs::CollisionObject &obj);
	void addDetectedObjectToLocalPlanningScene(const household_objects_database_msgs::DatabaseModelPoseList& model);

	// grasp execution
	virtual bool attemptGraspSequence(const std::string& group_name,const object_manipulator::GraspExecutionInfo& gei,bool performRecoveryMove = true);
	virtual bool attemptPlaceSequence(const std::string& group_name,const object_manipulator::PlaceExecutionInfo& pei);

	// demo monitoring
	void startCycleTimer();
	void printTiming();

	// general utilities
	static std::string makeCollisionObjectNameFromModelId(unsigned int model_id);
	const arm_navigation_msgs::CollisionObject* getCollisionObject(unsigned int model_id);

	// trajectory utilities
	virtual std::vector<std::string> getJointNames(const std::string& group);
	virtual trajectory_msgs::JointTrajectory getGripperTrajectory(int graspMove = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP);
	//virtual trajectory_msgs::JointTrajectory getGripperTrajectory(const std::string& arm_name,bool open);

	void printJointTrajectory(const trajectory_msgs::JointTrajectory &jt);
	bool validateJointTrajectory(trajectory_msgs::JointTrajectory &jt); // checks for null arrays and fill with zeros as needed

	// visual utilities
	void collisionObjToMarker(const arm_navigation_msgs::CollisionObject &obj, visualization_msgs::Marker &marker);
	void addMarker(std::string name,visualization_msgs::Marker &marker);
	void addMarker(std::string name,visualization_msgs::MarkerArray &marker);
	bool hasMarker(std::string);
	visualization_msgs::Marker& getMarker(std::string name);

	/* move sequence creation methods
	 * These methods generate all the necessary move steps corresponding to each manipulation sequence
	 */
	virtual bool createPickMoveSequence(const object_manipulation_msgs::PickupGoal &pickupGoal,
			const std::vector<object_manipulation_msgs::Grasp> &grasps_candidates,
			std::vector<object_manipulator::GraspExecutionInfo> &grasp_sequence,
			std::vector<object_manipulation_msgs::Grasp> &valid_grasps);
	virtual bool createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &placeGoal,
			const std::vector<geometry_msgs::PoseStamped> &place_poses,
			std::vector<object_manipulator::PlaceExecutionInfo> &place_sequence);


protected:

	// planning environemt
	planning_environment::CollisionModels cm_;

	// ids
	unsigned int current_planning_scene_id_;
	unsigned int last_mpr_id_;
	unsigned int max_mpr_id_;
	unsigned int max_trajectory_id_;

	TrajectoryExecutionDataVector last_trajectory_execution_data_vector_;

	// service clients
	ros::ServiceClient seg_srv_;
	ros::ServiceClient rec_srv_;
	ros::ServiceClient planning_service_client_;
	ros::ServiceClient trajectory_filter_service_client_;
	ros::ServiceClient object_database_model_mesh_client_;
	ros::ServiceClient grasp_planning_client;
	ros::ServiceClient object_database_model_description_client_;
	ros::ServiceClient set_planning_scene_diff_client_;

	// trajectory handlers and recorders
	boost::shared_ptr<TrajectoryRecorder> joint_state_recorder_;
	boost::shared_ptr<TrajectoryControllerHandler> arm_controller_handler_;
	boost::shared_ptr<TrajectoryControllerHandler> gripper_controller_handler_;

	trajectory_execution_monitor::TrajectoryExecutionMonitor trajectory_execution_monitor_;
	boost::function<bool(TrajectoryExecutionDataVector)> trajectories_finished_function_;
	boost::function<bool(TrajectoryExecutionDataVector)> grasp_action_finished_function_;

	// action services
	// will use grasp execution client to request pre-grasp action since the default gripper controller handler
	// ignores this step.
	boost::shared_ptr<GraspActionServerClient> grasp_exec_action_client_;

	// grasp move sequence generators
	//object_manipulator::PlaceTesterFast* place_tester_;
	//boost::shared_ptr<object_manipulator::GraspTesterFast> grasp_tester_;
	boost::shared_ptr<GraspSequenceValidator> grasp_tester_;
	boost::shared_ptr<PlaceSequenceValidator> place_tester_;

	arm_navigation_msgs::PlanningScene current_planning_scene_;
	arm_navigation_msgs::PlanningScene planning_scene_diff_;
	planning_models::KinematicState* current_robot_state_;

	boost::condition_variable execution_completed_;
	boost::mutex execution_mutex_;
	bool trajectories_succeeded_;

	// segmentation results
	tabletop_object_detector::TabletopSegmentation::Response segmentation_results_;
	std::vector<sensor_msgs::PointCloud> segmented_clusters_;
	arm_navigation_msgs::CollisionObject table_;

	// recognition results
	std::map<std::string, geometry_msgs::PoseStamped> recognized_obj_pose_map_;
	std::vector<household_objects_database_msgs::DatabaseModelPoseList> recognized_models_;
	household_objects_database_msgs::GetModelDescription::Response recognized_model_description_;

	// grasp planning results
	std::map<std::string, bool> object_in_hand_map_;
	std::map<std::string, std::string> current_grasped_object_name_;
	std::map<std::string, object_manipulation_msgs::Grasp> current_grasp_map_;
	object_manipulation_msgs::PickupGoal grasp_pickup_goal_;
	object_manipulation_msgs::PlaceGoal grasp_place_goal_;
	std::vector<object_manipulation_msgs::Grasp> grasp_candidates_;

	// pick/place move execution data
	std::vector<object_manipulator::GraspExecutionInfo> grasp_pick_sequence_;
	std::vector<object_manipulator::PlaceExecutionInfo> grasp_place_sequence_;

	//
	geometry_msgs::PoseStamped current_place_location_;

	// timing results
	ros::WallTime cycle_start_time_;
	ros::WallDuration execution_duration_;
	ros::WallDuration perception_duration_;
	ros::WallDuration planning_scene_duration_;
	ros::WallDuration motion_planning_duration_;
	ros::WallDuration trajectory_filtering_duration_;
	ros::WallDuration grasp_planning_duration_;

	// object publisher
	ros::Publisher attached_object_publisher_;

	// marker publishers
	ros::Publisher marker_array_publisher_;
	ros::Publisher marker_publisher_;

	// timer to publish markers periodically
	ros::Timer marker_pub_timer_;
	std::map<std::string,visualization_msgs::Marker> marker_map_;

	tf::TransformListener _TfListener;

	// ################################## ros parameters ##################################
	// group names
	std::string arm_group_name_;
	std::string gripper_group_name_;

	// link/gripper names
	std::string wrist_link_name_;
	std::string gripper_link_name_;

	// action services
	std::string grasp_action_service_;// = "/grasp_execution_action";
	std::string trajectory_action_service_;// = "/joint_trajectory_action";

	// topic names
	std::string path_planner_service_;
	std::string trajectory_filter_service_;
	std::string segmentation_service_;
	std::string recognition_service_;
	std::string grasp_planning_service_;
	std::string mesh_database_service_;
	std::string model_database_service_;
	std::string planning_scene_service_;
	std::string joint_states_topic_;

	// plugins
	std::string ik_plugin_name_;

	// pick/place
	double pick_approach_distance_;
	double place_approach_distance_;
	double place_retreat_distance_;

	// ################################## end of ros parameters ##################################
};

#endif /* ROBOTNAVIGATOR_H_ */
