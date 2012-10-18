/*
 * RobotPickPlaceNavigator.h
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

#ifndef ROBOTPICKPLACENAVIGATOR_H_
#define ROBOTPICKPLACENAVIGATOR_H_

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
#include <perception_tools/segmentation/SphereSegmentation.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction>  GraspActionServerClient;

using namespace trajectory_execution_monitor;

// ros param default values
const std::string DEFAULT_ARM_GROUP = "sia20d_arm";
const std::string DEFAULT_GRIPPER_GROUP = "end_effector";
const std::string DEFAULT_WRIST_LINK = "link_t";
const std::string DEFAULT_GRIPPER_LINK = "palm";
const std::string DEFAULT_GRASP_ACTION_SERVICE = "/grasp_execution_action";
const std::string DEFAULT_TRAJECTORY_ACTION_SERVICE = "/joint_trajectory_action";
const std::string DEFAULT_PATH_PLANNER_SERVICE = "/ompl_planning/plan_kinematic_path";
const std::string DEFAULT_SEGMENTATION_SERVICE = "/tabletop_segmentation";
const std::string DEFAULT_RECOGNITION_SERVICE = "/tabletop_object_recognition";
const std::string DEFAULT_TRAJECTORY_FILTER_SERVICE = "/trajectory_filter_server/filter_trajectory_with_constraints";
const std::string DEFAULT_GRASP_PLANNING_SERVICE = "/plan_point_cluster_grasp";
const std::string DEFAULT_MESH_DATABASE_SERVICE = "/objects_database_node/get_model_mesh";
const std::string DEFAULT_MODEL_DATABASE_SERVICE = "/objects_database_node/get_model_description";
const std::string DEFAULT_PLANNING_SCENE_SERVICE = "/environment_server/set_planning_scene_diff";
const std::string DEFAULT_IK_PLUGING = "SIA20D_Mesh_manipulator_kinematics/IKFastKinematicsPlugin";
const std::string DEFAULT_JOINT_STATES_TOPIC = "/joint_states";

// ros param names
const std::string PARAM_NAME_ARM_GROUP = "arm_group";
const std::string PARAM_NAME_GRIPPER_GROUP = "gripper_group";
const std::string PARAM_NAME_WRIST_LINK = "wrist_link";
const std::string PARAM_NAME_GRIPPER_LINK = "gripper_link";
const std::string PARAM_NAME_GRASP_ACTION_SERVICE = "grasp_action_service_name";
const std::string PARAM_NAME_TRAJECTORY_ACTION_SERVICE = "joint_trajectory_service_name";
const std::string PARAM_NAME_PATH_PLANNER_SERVICE = "planner_service_name";
const std::string PARAM_NAME_TRAJECTORY_FILTER_SERVICE = "trajectory_filter_service_name";
const std::string PARAM_NAME_SEGMENTATION_SERVICE = "segmentation_service_name";
const std::string PARAM_NAME_RECOGNITION_SERVICE = "recognition_service_name";
const std::string PARAM_NAME_GRASP_PLANNING_SERVICE = "grasp_planning_service_name";
const std::string PARAM_NAME_MESH_DATABASE_SERVICE = "mesh_database_service_name";
const std::string PARAM_NAME_MODEL_DATABASE_SERVICE = "model_database_service_name";
const std::string PARAM_NAME_PLANNING_SCENE_SERVICE = "planning_scene_service_name";
const std::string PARAM_NAME_IK_PLUGING = "arm_inverse_kinematics_plugin";
const std::string PARAM_NAME_JOINT_STATES_TOPIC = "joint_state_topic";

class RobotPickPlaceNavigator
{
public:

	struct GoalLocation
	{
	public:
		/*
		 * Determines how subsequent goal poses will be generated
		 */
		enum NextLocationGenerationMode
		{
			FIXED = 0,
			SHUFFLE = 1,
			SQUARE_ARRANGEMENT = 2,
			CIRCULAR_ARRANGEMENT = 3,
			SPIRAL_ARRANGEMENT = 4
		};

	public:
		GoalLocation()
		:FrameId("base_link"),
		 ChildFrameId("goal"),
		 GoalTransform(),
		 NumGoalCandidates(8),
		 Axis(0,0,1.0f),
		 NextLocationGenMode(FIXED),
		 MinObjectSpacing(0.05f),
		 MaxObjectSpacing(0.08f),
		 PlaceRegionRadius(0.20f)
		{
			tf::Vector3 pos = tf::Vector3(0.5,-0.48,0.1);
			GoalTransform.setOrigin(pos);
			GoalTransform.setRotation(tf::Quaternion::getIdentity());
		}
		~GoalLocation()
		{

		}

		void fetchParameters(std::string nameSpace)
		{
			double x = 0.0f,y = 0.0f,z = 0.0f, angle = GoalTransform.getRotation().getAngle();
			tf::Vector3 pos = GoalTransform.getOrigin(), axis = GoalTransform.getRotation().getAxis();

			// position parameters
			ros::param::param(nameSpace + "/position/x",x,pos.getX());
			ros::param::param(nameSpace + "/position/y",y,pos.getY());
			ros::param::param(nameSpace + "/position/z",z,pos.getZ());
			pos = tf::Vector3(x,y,z);

			// orientation parameters, should be provided in the form of angle-axis
			ros::param::param(nameSpace + "/orientation/axis/x",x,axis.getX());
			ros::param::param(nameSpace + "/orientation/axis/y",y,axis.getY());
			ros::param::param(nameSpace + "/orientation/axis/z",z,axis.getZ());
			ros::param::param(nameSpace + "/orientation/angle",angle,angle);
			axis = tf::Vector3(x,y,z);

			// goal frame
			ros::param::param(nameSpace + "/frame_id",FrameId,FrameId);
			ros::param::param(nameSpace + "/child_frame_id",ChildFrameId,ChildFrameId);

			tf::Transform t = tf::Transform(tf::Quaternion(axis,angle),pos);
			GoalTransform .setData(t);
			GoalTransform.frame_id_ = FrameId;
			GoalTransform.child_frame_id_ = ChildFrameId;

			// number of candidates
			ros::param::param(nameSpace + "/goal_candidates",NumGoalCandidates,NumGoalCandidates);
			if(NumGoalCandidates <= 0)
			{
				NumGoalCandidates = 1;
			}

			// axis for producing additional candidates
			ros::param::param(nameSpace + "/goal_rotation_axis/x",x,Axis.x());
			ros::param::param(nameSpace + "/goal_rotation_axis/y",y,Axis.y());
			ros::param::param(nameSpace + "/goal_rotation_axis/z",z,Axis.z());
			Axis = tf::Vector3(x,y,z).normalized();

			// next goal location generation parameters
			ros::param::param(nameSpace + "/next_location/generation_mode",(int&)NextLocationGenMode,(int&)NextLocationGenMode);
			ros::param::param(nameSpace + "/next_location/min_spacing",MinObjectSpacing,MinObjectSpacing);
			ros::param::param(nameSpace + "/next_location/max_spacing",MaxObjectSpacing,MaxObjectSpacing);
			ros::param::param(nameSpace + "/next_location/place_region_radius",PlaceRegionRadius,PlaceRegionRadius);
		}

		void generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses);

		std::string FrameId;
		std::string ChildFrameId;
		tf::StampedTransform GoalTransform;
		int NumGoalCandidates; // number of total goal transforms, each additional transform is produced by rotating
								// about an axis by a specified angle
		tf::Vector3 Axis; // used in producing additional goal candidates

		// Next goal position generation
		NextLocationGenerationMode NextLocationGenMode; // generation mode flag
		double MinObjectSpacing; // minimum distance between two objects inside goal region as measured from their local origins
		double MaxObjectSpacing; // maximum distance between two objects inside goal region as measured from their local origins
		double PlaceRegionRadius; // radius of circular region that contains all placed objects

	protected:
		std::vector<tf::Transform> previous_locations_;

		void createCandidatePosesByRotation(const tf::Transform &startTrans,int numCandidates,tf::Vector3 axis,
				std::vector<geometry_msgs::PoseStamped> &candidatePoses);

		void generateNextLocationShuffleMode(std::vector<geometry_msgs::PoseStamped> &placePoses);
		void generateNextLocationSquaredMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": Square place mode not implemented, using shuffle place mode");
			generateNextLocationShuffleMode(placePoses);
		}

		void generateNextLocationCircularMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": Circular place mode not implemented, using shuffle place mode");
			generateNextLocationShuffleMode(placePoses);
		}

		void generateNextLocationSpiralMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
		{
			ROS_WARN_STREAM(ros::this_node::getName()<<": Spiral place mode not implemented, using shuffle place mode");
			generateNextLocationShuffleMode(placePoses);
		}
	};

	struct JointConfiguration
	{
	public:
		JointConfiguration()
		:SideAngles()
		{
			SideAngles.push_back(-0.40f);
			SideAngles.push_back(0.8f);
			SideAngles.push_back(2.5f);
			SideAngles.push_back(1.5f);
			SideAngles.push_back(-0.5f);
			SideAngles.push_back(1.1f);
			SideAngles.push_back(0.6f);
		}

		~JointConfiguration()
		{

		}

		void fetchParameters(std::string nameSpace = "")
		{
			XmlRpc::XmlRpcValue list;
			if(ros::param::get(nameSpace + "/side_position",list))
			{
				if(list.getType()==XmlRpc::XmlRpcValue::TypeArray && list.size() > 0)
				{
					SideAngles.clear();
					for(int i = 0; i < list.size(); i++)
					{
						XmlRpc::XmlRpcValue &val = list[i];
						if(val.getType() == XmlRpc::XmlRpcValue::TypeDouble)
						{
							SideAngles.push_back(static_cast<double>(val));
						}
					}
				}
			}
		}

		std::vector<double> SideAngles;
	};

public:

	enum ConfigurationFlags
	{
		SETUP_FULL,
		SETUP_SPHERE_PICK_PLACE,
		SETUP_OTHER,
	};

	RobotPickPlaceNavigator(ConfigurationFlags flag = SETUP_SPHERE_PICK_PLACE);
	virtual ~RobotPickPlaceNavigator();


	// demo start
	void run();

protected:
	// setup
		void setup();
		void setupBallPickingDemo(); // will be removed
		void setupRecognitionOnly(); // will be removed

	// arm trajectory
		bool trajectoriesFinishedCallbackFunction(TrajectoryExecutionDataVector tedv);
		bool fastFilterTrajectory(const std::string& group_name,trajectory_msgs::JointTrajectory& jt);

	// household database
		bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
				arm_navigation_msgs::CollisionObject& obj,const geometry_msgs::PoseStamped& pose);

	// move arm methods
		bool moveArm(const std::string& group_name,const std::vector<double>& joint_positions);
		bool moveArmToSide();
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

	// callbacks
		void callbackPublishMarkers(const ros::TimerEvent &evnt);

	// ros parameters
		void fetchParameters(std::string nameSpace = "");

	// run demos
		void runFullPickPlace();
		void runSpherePickPlace();

	// new methods

		/* Service methods
		 * these methods perform the following:
		 * 	-  Build request objects,
		 * 	-  Check results
		 * 	-  Store results needed for subsequent steps in the manipulation
		 * 	-  Perform additional post processing stuff such as updating the planning scene or publish data
		 */
		bool performSegmentation();
		bool performSphereSegmentation();// operates on the results produced by performSegmentation
		bool performRecognition();// operates on the results produced by performSegmentation
		bool performGraspPlanning();
//		bool performPathPlanning();
//		bool performTrajectoryFilter();
//		bool performPlanningSceneSUpdate(); // this method may not be needed

		/* move sequence creation methods
		 * These methods generate all the necessary move steps corresponding to each manipulation sequence
		 */
		void createPickMoveSequence(const object_manipulation_msgs::PickupGoal &pickupGoal,
				const std::vector<object_manipulation_msgs::Grasp> &grasps,
				std::vector<object_manipulator::GraspExecutionInfo> &graspSequence);
		void createPlaceMoveSequence(const object_manipulation_msgs::PlaceGoal &placeGoal,
				const std::vector<geometry_msgs::PoseStamped> &placePoses,
				std::vector<object_manipulator::PlaceExecutionInfo> &placeSequence);

		// will be removed since GoalLocation struct already does this
		void createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses);

protected:

	// setup flag
	ConfigurationFlags configuration_type_;

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

	  tf::TransformListener _TfListener;

	  // segmentation
	  SphereSegmentation _SphereSeg;

	  // ################################## ros parameters ##################################
	  GoalLocation _GoalParameters;
	  JointConfiguration _JointConfigurations;

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

#endif /* ROBOTPICKPLACENAVIGATOR_H_ */
