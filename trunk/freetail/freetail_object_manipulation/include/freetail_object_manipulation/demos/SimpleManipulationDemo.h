/*
 * SimpleManipulationDemo.h
 *
 *  Created on: Jul 11, 2012
 *      Author: coky
 */

#ifndef SIMPLEMANIPULATIONDEMO_H_
#define SIMPLEMANIPULATIONDEMO_H_

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
#include <freetail_object_manipulation/utils/grasp_posture_trajectory_controller_handler.h>
#include <freetail_object_manipulation/utils/CustomPlaceTester.h>
#include <tf/transform_listener.h>

using namespace trajectory_execution_monitor;

class RosParamsList
{
public:
	// service names
	struct Names
	{
		// service names
		static const std::string PathPlannerService;
		static const std::string TrajectoryFilterService;
		static const std::string SegmentationService;
		static const std::string RecognitionService;
		static const std::string GraspPlanningService;
		static const std::string MeshDatabaseService;
		static const std::string ModelDatabaseService;
		static const std::string PlanningSceneService;

		// plugin names
		static const std::string InverseKinematicsPlugin;
	};

	struct Values
	{
		// service names
		static std::string PathPlannerService;
		static std::string TrajectoryFilterService;
		static std::string SegmentationService;
		static std::string RecognitionService;
		static std::string GraspPlanningService;
		static std::string MeshDatabaseService;
		static std::string ModelDatabaseService;
		static std::string PlanningSceneService;

		// plugin names
		static std::string InverseKinematicsPlugin;
	};

public:

	RosParamsList(){};


	static void fetchParams(ros::NodeHandle &hn,bool useNodeNamespace = true);
	static void fetchParams(bool useNodeNamespace = true);

};

class SimpleManipulationDemo {
public:
	SimpleManipulationDemo();
	virtual ~SimpleManipulationDemo();

	// methods
	void setup();
	void setupRecognitionOnly();
	bool trajectoriesFinishedCallbackFunction(TrajectoryExecutionDataVector tedv);
	void revertPlanningScene();
	std::vector<std::string> getJointNames(const std::string& group);
	void updateCurrentJointStateToLastTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj);
	bool getAndSetPlanningScene();
	bool moveArm(const std::string& group_name,const std::vector<double>& joint_positions);
	void printJointTrajectory(const trajectory_msgs::JointTrajectory &jt);
	bool validateJointTrajectory(trajectory_msgs::JointTrajectory &jt); // checks for null arrays
	bool fastFilterTrajectory(const std::string& group_name,trajectory_msgs::JointTrajectory& jt);
	bool moveArmToSide();
	void addDetectedTableToPlanningSceneDiff(const tabletop_object_detector::Table &table);
	void addDetectedObjectToPlanningSceneDiff(const household_objects_database_msgs::DatabaseModelPoseList& model);
	bool segmentAndRecognize();
	bool recognize();
	bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,arm_navigation_msgs::CollisionObject& obj,const geometry_msgs::PoseStamped& pose);
	bool selectGraspableObject(const std::string& arm_name,object_manipulation_msgs::PickupGoal& pickup_goal,std::vector<object_manipulation_msgs::Grasp>& grasps);
	bool putDownSomething(const std::string& arm_name);
	bool pickUpSomething(const std::string& arm_name);
	void attachCollisionObjectCallback(const std::string& group_name);
	void detachCollisionObjectCallback(const std::string& group_name);
	bool attemptGraspSequence(const std::string& group_name,const object_manipulator::GraspExecutionInfo& gei);
	bool attemptPlaceSequence(const std::string& group_name,const object_manipulator::PlaceExecutionInfo& pei);
	bool getObjectGrasps(const std::string& arm_name,const household_objects_database_msgs::DatabaseModelPose& dmp,const sensor_msgs::PointCloud& cloud,std::vector<object_manipulation_msgs::Grasp>& grasps);
	trajectory_msgs::JointTrajectory getGripperTrajectory(const std::string& arm_name,bool open);
	bool attachCollisionObject(const std::string& group_name,const std::string& collision_object_name, const object_manipulation_msgs::Grasp& grasp);
	bool detachCollisionObject(const std::string& group_name,const geometry_msgs::PoseStamped& place_pose,const object_manipulation_msgs::Grasp& grasp);
	void startCycleTimer();
	void printTiming();

	void runDemo();

	// utilities
	static std::string makeCollisionObjectNameFromModelId(unsigned int model_id);
	const arm_navigation_msgs::CollisionObject* getCollisionObject(unsigned int model_id);

protected:
	// members

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
	  ros::ServiceClient grasp_planning_client;
	  ros::ServiceClient object_database_model_description_client_;

	  ros::ServiceClient set_planning_scene_diff_client_;

	  boost::shared_ptr<TrajectoryRecorder> joint_state_recorder_;

	  boost::shared_ptr<TrajectoryControllerHandler> arm_controller_handler_;
	  boost::shared_ptr<TrajectoryControllerHandler> gripper_controller_handler_;

	  trajectory_execution_monitor::TrajectoryExecutionMonitor trajectory_execution_monitor_;
	  boost::function<bool(TrajectoryExecutionDataVector)> trajectories_finished_function_;

	  object_manipulator::GraspTesterFast* grasp_tester_;
	  CustomPlaceTester *place_tester_;
	  //object_manipulator::PlaceTesterFast* place_tester_;

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

	  tf::TransformListener _TfListener;
};

#endif /* SIMPLEMANIPULATIONDEMO_H_ */
