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
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction>  GraspActionServerClient;

using namespace trajectory_execution_monitor;

class RobotNavigator {
public:
	RobotNavigator();
	virtual ~RobotNavigator();

	// demo start
	virtual void run();

protected:
	// setup
	virtual void setup();

	// ros parameters
	void fetchParameters(std::string nameSpace = "");

	/* Service methods
	 * these methods perform the following:
	 * 	-  Build request objects,
	 * 	-  Check results
	 * 	-  Store results needed for subsequent steps in the manipulation
	 * 	-  Perform additional post processing stuff such as updating the planning scene or publish data
	 */
	virtual bool performSegmentation();
	virtual bool performRecognition();
	virtual bool performGraspPlanning();
	virtual bool performTrajectoryFiltering(const std::string& group_name,trajectory_msgs::JointTrajectory& jt);

	// goal position: this should be implemented by each specific specialization of the class
	virtual void createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses) = 0;

	// callbacks
	virtual void callbackPublishMarkers(const ros::TimerEvent &evnt);
	virtual bool trajectoriesFinishedCallbackFunction(TrajectoryExecutionDataVector tedv);

	// database comm
	bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
			arm_navigation_msgs::CollisionObject& obj,const geometry_msgs::PoseStamped& pose);

	// move arm methods
	virtual bool moveArmToSide(); // it is preferable to implement this in order to tell the robot arm where "side" is
	bool moveArm(const std::string& group_name,const std::vector<double>& joint_positions);
	bool moveArmThroughPickSequence();
	bool moveArmThroughPlaceSequence();

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
	bool getAndSetPlanningScene();

	// add objects to local copy of the planning scene, however the planning scene service needs to be called in order to push
	// any changes into the actual planning scene
	void addDetectedTableToPlanningSceneDiff(const tabletop_object_detector::Table &table);
	void addDetectedObjectToPlanningSceneDiff(arm_navigation_msgs::CollisionObject &obj);
	void addDetectedObjectToPlanningSceneDiff(const household_objects_database_msgs::DatabaseModelPoseList& model);

	// grasp execution
	bool attemptGraspSequence(const std::string& group_name,const object_manipulator::GraspExecutionInfo& gei);
	bool attemptPlaceSequence(const std::string& group_name,const object_manipulator::PlaceExecutionInfo& pei);

	// demo monitoring
	void startCycleTimer();
	void printTiming();

	// utilities
	static std::string makeCollisionObjectNameFromModelId(unsigned int model_id);
	const arm_navigation_msgs::CollisionObject* getCollisionObject(unsigned int model_id);
	void printJointTrajectory(const trajectory_msgs::JointTrajectory &jt);
	bool validateJointTrajectory(trajectory_msgs::JointTrajectory &jt); // checks for null arrays and fill with zeros as needed
	std::vector<std::string> getJointNames(const std::string& group);
	void collisionObjToMarker(const arm_navigation_msgs::CollisionObject &obj, visualization_msgs::Marker &marker);
	trajectory_msgs::JointTrajectory getGripperTrajectory(const std::string& arm_name,bool open);

	/* move sequence creation methods
	 * These methods generate all the necessary move steps corresponding to each manipulation sequence
	 */
	void createPickMoveSequence(const object_manipulation_msgs::PickupGoal &pickupGoal,
			const std::vector<object_manipulation_msgs::Grasp> &grasps,
			std::vector<object_manipulator::GraspExecutionInfo> &graspSequence);
	void createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &placeGoal,
			const std::vector<geometry_msgs::PoseStamped> &placePoses,
			std::vector<object_manipulator::PlaceExecutionInfo> &placeSequence);

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
	//ros::ServiceClient trajectory_filter_fast_service_client_;
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

	// action services
	// will use grasp execution client to request pre-grasp action since the default gripper controller handler
	// ignores this step.
	boost::shared_ptr<GraspActionServerClient> grasp_exec_action_client_;

	// grasp move sequence generators
	object_manipulator::GraspTesterFast* grasp_tester_;
	PlaceSequenceValidator *place_tester_;
	//object_manipulator::PlaceTesterFast* place_tester_;

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

	ros::Publisher attached_object_publisher_;

	// marker publishers
	ros::Publisher vis_marker_array_publisher_;
	ros::Publisher vis_marker_publisher_;

	// timer to publish markers periodically
	ros::Timer _MarkerPubTimer;
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

	// ################################## end of ros parameters ##################################
};

#endif /* ROBOTNAVIGATOR_H_ */
