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
#include <freetail_object_manipulation/segmentation/SphereSegmentation.h>
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


	static void fetchParams(std::string nameSpace = "");
};

struct GoalLocation
{
public:
	GoalLocation()
	:FrameId("base_link"),
	 ChildFrameId("goal"),
	 GoalTransform(),
	 NumGoalCandidates(8),
	 Axis(0,0,1.0f)
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
	}

	std::string FrameId;
	std::string ChildFrameId;
	tf::StampedTransform GoalTransform;
	int NumGoalCandidates; // number of total goal transforms, each additional transform is produced by rotating
							// about an axis by a specified angle
	tf::Vector3 Axis; // used in producing additional goal candidates

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

class SimpleManipulationDemo {
public:
	SimpleManipulationDemo();
	virtual ~SimpleManipulationDemo();

	// setup
	void setup();
	void setupBallPickingDemo();
	void setupRecognitionOnly();

	// arm trajectory
	bool trajectoriesFinishedCallbackFunction(TrajectoryExecutionDataVector tedv);
	bool fastFilterTrajectory(const std::string& group_name,trajectory_msgs::JointTrajectory& jt);

	// segmentation/recognition
	bool segmentAndRecognize();
	bool segmentSpheres();
	bool recognize();

	// household database
	bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,arm_navigation_msgs::CollisionObject& obj,const geometry_msgs::PoseStamped& pose);

	// grasp planning
	bool selectGraspableObject(const std::string& arm_name,object_manipulation_msgs::PickupGoal& pickup_goal,std::vector<object_manipulation_msgs::Grasp>& grasps);
	bool getObjectGrasps(const std::string& arm_name,const household_objects_database_msgs::DatabaseModelPose& dmp,const sensor_msgs::PointCloud& cloud,std::vector<object_manipulation_msgs::Grasp>& grasps);
	trajectory_msgs::JointTrajectory getGripperTrajectory(const std::string& arm_name,bool open);

	// arm command
	bool putDownSomething(const std::string& arm_name);
	bool placeAtGoalLocation(const std::string &armName);
	bool pickUpSomething(const std::string& arm_name);
	bool moveArm(const std::string& group_name,const std::vector<double>& joint_positions);
	bool moveArmToSide();

	// planning scene
	void attachCollisionObjectCallback(const std::string& group_name);
	void detachCollisionObjectCallback(const std::string& group_name);
	bool attachCollisionObject(const std::string& group_name,const std::string& collision_object_name, const object_manipulation_msgs::Grasp& grasp);
	bool detachCollisionObject(const std::string& group_name,const geometry_msgs::PoseStamped& place_pose,const object_manipulation_msgs::Grasp& grasp);
	void revertPlanningScene();
	void updateCurrentJointStateToLastTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj);
	bool getAndSetPlanningScene();
	void addDetectedTableToPlanningSceneDiff(const tabletop_object_detector::Table &table);
	void addDetectedObjectToPlanningSceneDiff(arm_navigation_msgs::CollisionObject &obj);
	void addDetectedObjectToPlanningSceneDiff(const household_objects_database_msgs::DatabaseModelPoseList& model);

	// grasp execution
	bool attemptGraspSequence(const std::string& group_name,const object_manipulator::GraspExecutionInfo& gei);
	bool attemptPlaceSequence(const std::string& group_name,const object_manipulator::PlaceExecutionInfo& pei);

	// demo monitoring
	void startCycleTimer();
	void printTiming();

	// demo
	void runSimpleManipulationDemo();
	void runBallPickingDemo();

	// utilities
	static std::string makeCollisionObjectNameFromModelId(unsigned int model_id);
	const arm_navigation_msgs::CollisionObject* getCollisionObject(unsigned int model_id);
	void printJointTrajectory(const trajectory_msgs::JointTrajectory &jt);
	bool validateJointTrajectory(trajectory_msgs::JointTrajectory &jt); // checks for null arrays
	std::vector<std::string> getJointNames(const std::string& group);
	void collisionObjToMarker(const arm_navigation_msgs::CollisionObject &obj, visualization_msgs::Marker &marker);

	// callbacks
	void callbackPublishMarkers(const ros::TimerEvent &evnt);

protected:
	// members

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
	  actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> grasp_exec_action_client_;

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

	  // marker publishers
	  ros::Publisher vis_marker_array_publisher_;
	  ros::Publisher vis_marker_publisher_;

	  // timer to publish markers periodically
	  ros::Timer _MarkerPubTimer;

	  tf::TransformListener _TfListener;

	  // segmentation
	  SphereSegmentation _SphereSeg;

	  // parameters
	  GoalLocation _GoalParameters;
	  JointConfiguration _JointConfigurations;
};

#endif /* SIMPLEMANIPULATIONDEMO_H_ */
