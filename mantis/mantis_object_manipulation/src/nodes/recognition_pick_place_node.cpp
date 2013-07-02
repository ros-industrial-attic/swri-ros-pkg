/*
 * recognition_pick_place_node.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: ros developer 
 */

#include <mantis_object_manipulation/arm_navigators/SortClutterArmNavigator.h>
#include <map>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <iostream>

// topics
const static std::string ROTATION_CORRECTION_TOPIC = "rotation_correction";

// parameters
const static std::string PARAM_JOINT_INTERMEDIATE_POSITION = "joint_intermediate_position";
const static std::string PARAM_JOINT_HOME_POSITION = "joint_home_position";
const static std::string PARAM_ALTERNATE_DROPOFF_GOAL = "alternate_dropoff";

namespace selection
{
	using namespace boost::assign;

	enum Selection
	{
		EXIT = 0,
		PROCEED = 1
	};

	static const std::map<std::string,int> SelectionMap = boost::assign::map_list_of("EXIT",(int)EXIT)
		("PROCEED",PROCEED);
}


class RecognitionPickNavigator: public SortClutterArmNavigator
{
public:

	RecognitionPickNavigator()
	{

	}

	virtual ~RecognitionPickNavigator()
	{

	}

	virtual void run()
	{
		ros::NodeHandle nh;
		bool rotation_correction_succeeded_ = true;
		ros::AsyncSpinner spinner(4);
		spinner.start();
		srand(time(NULL));

		// arm setup
		setup();

		// cycle
		while(ros::ok())
		{
			moveArmHome();

			std::cout<<"\n\tRecognition request prompt: ";
			if(!promptUser())
			{
				ROS_INFO_STREAM("Exiting node");
				break;
			}

			ROS_INFO_STREAM("Recognition started");
			if(performRecognition())
			{
				ROS_INFO_STREAM("Recognition succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Recognition failed");
				break;
			}

			ROS_INFO_STREAM("Grasp planning started");
			if(performPickGraspPlanning())
			{
				ROS_INFO_STREAM("Grasp planning succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Grasp planning failed");
				break;
			}

			// moving to pre-grasp position and wait
			ROS_INFO_STREAM("Moving to Pre-grasp");
			if(moveToPregraspPose())
			{
				ROS_INFO_STREAM("Move to Pre-grasp succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Move to Pre-grasp failed");
				moveArmHome();
				break;
			}

			// waiting for user input
			std::cout<<"\n\tMove to pick prompt: ";
			if(!promptUser())
			{
				ROS_INFO_STREAM("Exiting node");
				break;
			}

			// moving for pickup in cluttered zone
			ROS_INFO_STREAM("Pickup started");
			if(moveArmThroughPickSequence())
			{
				ROS_INFO_STREAM("Pickup completed");
			}
			else
			{
				ROS_WARN_STREAM("Pickup failed");
				moveArmHome();
				break;
			}

			// resetting rotation correction transform
			rot_correction_tf_ = tf::Transform::getIdentity();
			rotation_correction_succeeded_ = false;

			std::cout<<"\n\tMove to intermediate position prompt:";
			if(promptUser())
			{
				if(updateChangesToPlanningScene() && moveArm(arm_group_name_,joint_intermediate_conf_.SideAngles))
				{
					ROS_INFO_STREAM("Move to intermediate position succeeded");
				}
				else
				{
					ROS_ERROR_STREAM("Move to intermediate position failed");
					moveArmHome();
					break;
				}

				// calling rectification imaging service
				ROS_INFO_STREAM("Calling '" + ROTATION_CORRECTION_TOPIC + "' service");
				double angle = 0.0f;
				mantis_perception::mantis_recognition::Request req;
				mantis_perception::mantis_recognition::Response res;
				if(rotation_correction_client_.call(req,res) && (res.pose.rotation >= 0.0f))
				{
					rotation_correction_succeeded_ = true;
					angle = static_cast<double>(M_PI * res.pose.rotation)/180.0f;
					rot_correction_tf_.setRotation(tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),angle));
					ROS_INFO_STREAM("'" << ROTATION_CORRECTION_TOPIC <<
							"' service succeeded with angle: "<<res.pose.rotation<<"( "<<angle <<" radians)");

				}
				else
				{
					rotation_correction_succeeded_ = false;
					ROS_ERROR_STREAM("'" + ROTATION_CORRECTION_TOPIC + "' service failed");
				}
			}
			else
			{
				ROS_WARN_STREAM("skipping intermediate position");

			}

			// waiting for user input
			std::cout<<"\n\tMove to place prompt: ";
			if(!promptUser())
			{
				ROS_INFO_STREAM("Exiting node");
				break;
			}

			// moving for place in singulated zone
			ROS_INFO_STREAM("Place started");
			if( (rotation_correction_succeeded_ ? performPlaceGraspPlanning(singulated_dropoff_location_) : performPlaceGraspPlanning(alternate_dropoff_location_))
					&& RobotNavigator::moveArmThroughPlaceSequence())
			{
				ROS_INFO_STREAM("Place completed");
			}
			else
			{
				ROS_WARN_STREAM("Place failed");
				moveArmHome();
				break;
			}

		}

	}

protected:

	virtual void setup()
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
			//recognition
			recognition_client_ = nh.serviceClient<mantis_perception::mantis_recognition>(recognition_service_,true);
			recognition_client_.waitForExistence();

			// rotation correction
			rotation_correction_client_ = nh.serviceClient<mantis_perception::mantis_recognition>(nh.getNamespace() + "/" + ROTATION_CORRECTION_TOPIC,true);
			rotation_correction_client_.waitForExistence();

			// path planning
			planning_service_client_ = nh.serviceClient<arm_navigation_msgs::GetMotionPlan>(path_planner_service_);

			// trajectory filter
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
			marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC,1);
			attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);

			// setting up timer obj
			marker_pub_timer_ = nh.createTimer(ros::Duration(0.4f),&RecognitionPickNavigator::callbackPublishMarkers,this);

			ROS_INFO_STREAM(NODE_NAME<<": Setting up dynamic libraries");

			// trajectory generators
			grasp_tester_ = GraspTesterPtr(new GraspSequenceValidator(&cm_, ik_plugin_name_));
			place_tester_ = PlaceSequencePtr(new PlaceSequenceValidator(&cm_, ik_plugin_name_));

			// trajectory callbacks
			trajectories_finished_function_ = boost::bind(&RecognitionPickNavigator::trajectoryFinishedCallback, this, true,_1);
			grasp_action_finished_function_ = boost::bind(&RecognitionPickNavigator::trajectoryFinishedCallback, this, false,_1);

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
			addMarker(MARKER_SEGMENTED_OBJECT,marker);
			addMarker(MARKER_ATTACHED_OBJECT,marker);

			// pick and place zone markers
			updateMarkerArrayMsg();
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

		ROS_INFO_STREAM(NODE_NAME<<": Finished setup");
	}

	void updateMarkerArrayMsg()
	{
		typedef std::vector<visualization_msgs::Marker>::iterator Iter;
		boost::mutex::scoped_lock lock(marker_array_mutex_);
		{
			for(Iter i = marker_array_msg_.markers.begin(); i != marker_array_msg_.markers.end(); i++)
			{
				visualization_msgs::Marker &marker = *i;
				marker.action = visualization_msgs::Marker::DELETE;
			}
			marker_array_pub_.publish(marker_array_msg_);
			marker_array_msg_.markers.clear();
		}
	}

	virtual void fetchParameters(std::string name_space = "")
	{
		ros::NodeHandle nh("~");

		AutomatedPickerRobotNavigator::fetchParameters(name_space);
		joint_home_conf_.fetchParameters(nh.getNamespace() + "/" + PARAM_JOINT_HOME_POSITION);
		joint_intermediate_conf_.fetchParameters(nh.getNamespace() + "/" + PARAM_JOINT_INTERMEDIATE_POSITION);
		singulated_dropoff_location_.fetchParameters(singulated_dropoff_ns_);
		alternate_dropoff_location_.fetchParameters(nh.getNamespace() + "/" + PARAM_ALTERNATE_DROPOFF_GOAL);
	}

	virtual bool performRecognition()
	{
		return AutomatedPickerRobotNavigator::performRecognition();
	}

	virtual bool performPickGraspPlanning()
	{
		//  clearing previous data
		grasp_pickup_goal_.target.potential_models.clear();
		grasp_candidates_.clear();

		// gathering latest recognition results
		household_objects_database_msgs::DatabaseModelPose &modelPose = recognized_models_[0].model_list[0];
		std::string modelId = makeCollisionObjectNameFromModelId(modelPose.model_id);

		// generating single grasp
		grasp_candidates_.push_back(object_manipulation_msgs::Grasp());
		grasp_candidates_[0].desired_approach_distance = pick_approach_distance_;
		grasp_candidates_[0].min_approach_distance = pick_approach_distance_;
		grasp_candidates_[0].grasp_pose = candidate_pick_poses_[0].pose; // pose of tcp relative to world

		// instantiating needed transforms and poses
		tf::StampedTransform wrist_in_tcp_tf = tf::StampedTransform();
		tf::Transform object_in_world_tf;
		tf::Transform object_in_world_inverse_tf;

		// filling transforms
		tf::poseMsgToTF(modelPose.pose.pose, object_in_world_tf);
		object_in_world_inverse_tf = object_in_world_tf.inverse();
		_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);

		// applying transformation to grasp pose so that the arm wrist relative to the object is obtained
		for(unsigned int i = 0; i < grasp_candidates_.size(); i++)
		{
			tf::Transform tcp_in_world_tf;
			tf::poseMsgToTF(grasp_candidates_[i].grasp_pose, tcp_in_world_tf);
			tf::poseTFToMsg(object_in_world_inverse_tf*(tcp_in_world_tf*wrist_in_tcp_tf),
				  grasp_candidates_[i].grasp_pose);
		}

		// storing grasp pickup goal to be used later during pick move sequence execution
		grasp_pickup_goal_.collision_object_name = modelId;
		grasp_pickup_goal_.target.reference_frame_id = modelId;
		grasp_pickup_goal_.allow_gripper_support_collision = true;
		grasp_pickup_goal_.target.potential_models.push_back(modelPose);

		// generating grasp pick sequence
		updateChangesToPlanningScene();
		grasp_pick_sequence_.clear();
		std::vector<object_manipulation_msgs::Grasp> valid_grasps;
		ROS_INFO_STREAM(NODE_NAME<<": Testing "<<grasp_candidates_.size()<<" grasp candidates");
		if(!createPickMoveSequence(grasp_pickup_goal_,grasp_candidates_,grasp_pick_sequence_,valid_grasps))
		{
			ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp pick sequence");
			return false;
		}
		grasp_candidates_.assign(valid_grasps.begin(),valid_grasps.end());

		// updating gripper in local planning scene
		tf::Transform first_grasp_in_world_tf;
		tf::poseMsgToTF(grasp_candidates_[0].grasp_pose, first_grasp_in_world_tf);
		planning_models::KinematicState state(*current_robot_state_);
		state.updateKinematicStateWithLinkAt(gripper_link_name_, first_grasp_in_world_tf);

		// printing completion info message
		ROS_INFO_STREAM(NODE_NAME<<": Grasp Position in "<< cm_.getWorldFrameId()
				<<" is: "<<grasp_candidates_[0].grasp_pose.position.x << " "
				  << grasp_candidates_[0].grasp_pose.position.y << " "
				  << grasp_candidates_[0].grasp_pose.position.z);

		return true;
	}

	virtual bool performPlaceGraspPlanning(GoalLocation &goal)
	{
		using namespace mantis_object_manipulation;
		tf::Transform generated_object_tf;
		geometry_msgs::PoseStamped generated_object_pose;

		// applying orientation correction to object pose
		candidate_place_poses_.clear();
		generated_object_pose.header.frame_id = goal.GoalTransform.frame_id_;
		tf::poseTFToMsg(goal.GoalTransform * rot_correction_tf_,generated_object_pose.pose);
		tf::poseMsgToTF(generated_object_pose.pose,generated_object_tf);
		candidate_place_poses_.push_back(generated_object_pose);


		// printing object location results
		ROS_INFO_STREAM("Generated "<<candidate_place_poses_.size()
				<<" candidates with location at:  "<<generated_object_tf.getOrigin().x() <<
				", "<<generated_object_tf.getOrigin().y()<<", "<<generated_object_tf.getOrigin().z());
		ROS_INFO_STREAM("Generated object goal angle "<<generated_object_tf.getRotation().getAngle()
				<<"( "<<180.0f *generated_object_tf.getRotation().getAngle()/M_PI<<" degrees)");
		ROS_INFO_STREAM("Generated object goal axis: "<<generated_object_tf.getRotation().getAxis().getX()
						<<", "<<generated_object_tf.getRotation().getAxis().getY()
						<<", "<<generated_object_tf.getRotation().getAxis().getZ()
						<<" ");


		//	updating grasp place goal data
		grasp_place_goal_.arm_name = arm_group_name_;
		grasp_place_goal_.approach.direction.header.frame_id = cm_.getWorldFrameId();
		grasp_place_goal_.collision_object_name = "attached_"+current_grasped_object_name_[arm_group_name_];

		// finding valid grasp place sequence
		geometry_msgs::Pose tcp_in_objct_pose;

		// finding pose of wrist relative to object
		updateChangesToPlanningScene();
		tf::StampedTransform wrist_in_tcp_tf = tf::StampedTransform(), wrist_in_obj_tf = tf::StampedTransform();
		_TfListener.lookupTransform(gripper_link_name_,wrist_link_name_,ros::Time(0),wrist_in_tcp_tf);

		// storing tcp in object pose in grasp place goal
		tf::poseMsgToTF(grasp_candidates_[0].grasp_pose,wrist_in_obj_tf);
		tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),grasp_place_goal_.grasp.grasp_pose);
		//tf::poseTFToMsg(wrist_in_obj_tf*(wrist_in_tcp_tf.inverse()),tcp_in_objct_pose);
		//manipulation_utils::rectifyPoseZDirection(tcp_in_objct_pose,PLACE_RECTIFICATION_TF,grasp_place_goal_.grasp.grasp_pose);

		grasp_place_sequence_.clear();
		if(!createPlaceMoveSequence(grasp_place_goal_,candidate_place_poses_,grasp_place_sequence_))
		{
			ROS_ERROR_STREAM(NODE_NAME<<": Failed to create valid grasp place sequence");
			return false;
		}

		return true;
	}

	virtual bool moveArmHome()
	{
		return updateChangesToPlanningScene() && moveArm(arm_group_name_,joint_home_conf_.SideAngles);
	}

	bool moveLastJointByAngle(const JointConfiguration &j,double angle)
	{
		std::vector<double> joint_vals(j.SideAngles.begin(),j.SideAngles.end());
		ROS_INFO_STREAM("Start Angle: "<< joint_vals.back());
		joint_vals[joint_vals.size()-1] = joint_vals.back() + angle;
		ROS_INFO_STREAM("End Angle: "<< joint_vals.back());
		return updateChangesToPlanningScene() && moveArm(arm_group_name_,joint_vals);
	}

	bool moveToPregraspPose()
	{
		// moving to pre-grasp configuration
		return moveArm(arm_group_name_,grasp_pick_sequence_[0].approach_trajectory_.points[0].positions);
	}

	virtual void callbackPublishMarkers(const ros::TimerEvent &evnt)
	{
		AutomatedPickerRobotNavigator::callbackPublishMarkers(evnt);
	}

	bool promptUser()
	{
		int user_entry=selection::PROCEED;
		bool proceed = true;

		std::cout<<"\n\tSelect one of the following options:\n";
		for(std::map<std::string,int>::const_iterator i = selection::SelectionMap.begin()
				; i != selection::SelectionMap.end(); i++)
		{
			std::cout<<"\t\t( "<<i->second <<") - "<<i->first<<"\n";
		}

		std::cout<<"\n\tEnter Selection : ";
		if(std::cin >> user_entry)
		{
			switch(user_entry)
			{
			case selection::PROCEED:

				proceed = true;
				break;

			case selection::EXIT:

				proceed = false;
				break;
			}
		}
		else
		{
			ROS_ERROR_STREAM("Unable to get user input, exiting");
			proceed = false;
		}

		return proceed;
	}

protected:

	// ros services
	ros::ServiceClient rotation_correction_client_;

	// joint positions
	JointConfiguration joint_home_conf_;
	JointConfiguration joint_intermediate_conf_;

	// place goals
	GoalLocation alternate_dropoff_location_;

	// poses/transforms
	tf::Transform rot_correction_tf_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"recognition_pick_place_node");

	RobotNavigator::Ptr navigator_ptr = RobotNavigator::Ptr(new RecognitionPickNavigator());
	navigator_ptr->run();
	exit(EXIT_SUCCESS);

	return 0;
}
