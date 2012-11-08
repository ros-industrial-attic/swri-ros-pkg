/*
 * SpherePickingRobotNavigator.cpp
 *
 *  Created on: Oct 19, 2012
 */

#include <mantis_object_manipulation/arm_navigators/SpherePickingRobotNavigator.h>

std::string SpherePickingRobotNavigator::MARKER_SEGMENTED_OBJECT = "segmented_obj";
std::string SpherePickingRobotNavigator::SEGMENTATION_NAMESPACE = "segmentation";
std::string SpherePickingRobotNavigator::GOAL_NAMESPACE = "goal";
std::string SpherePickingRobotNavigator::JOINT_CONFIGURATIONS_NAMESPACE = "joints";

SpherePickingRobotNavigator::SpherePickingRobotNavigator()
:RobotNavigator()
{
	// TODO Auto-generated constructor stub
	GOAL_NAMESPACE = NODE_NAME + "/" + GOAL_NAMESPACE;
	SEGMENTATION_NAMESPACE = NODE_NAME + "/" + SEGMENTATION_NAMESPACE;
	JOINT_CONFIGURATIONS_NAMESPACE = NODE_NAME + "/" + JOINT_CONFIGURATIONS_NAMESPACE;

}

SpherePickingRobotNavigator::~SpherePickingRobotNavigator()
{
	// TODO Auto-generated destructor stub
	ROS_INFO_STREAM(NODE_NAME<<": Exiting navigator");
}

void SpherePickingRobotNavigator::setup()
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
		// segmentation
		seg_srv_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(segmentation_service_, true);

		// path and grasp planning
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
		marker_publisher_ = nh.advertise<visualization_msgs::Marker> (VISUALIZATION_TOPIC, 128);
		attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);

		// setting up timer obj
		marker_pub_timer_ = nh.createTimer(ros::Duration(0.4f),&SpherePickingRobotNavigator::callbackPublishMarkers,this);

		ROS_INFO_STREAM(NODE_NAME<<": Setting up dynamic libraries");
		// others
		grasp_tester_ = GraspTesterPtr(new object_manipulator::GraspTesterFast(&cm_, ik_plugin_name_));
		place_tester_ = PlaceSequencePtr(new PlaceSequenceValidator(&cm_, ik_plugin_name_));
//		grasp_tester_ = new object_manipulator::GraspTesterFast(&cm_, ik_plugin_name_);
//		place_tester_ = new PlaceSequenceValidator(&cm_, ik_plugin_name_);
		trajectories_finished_function_ = boost::bind(&SpherePickingRobotNavigator::trajectoriesFinishedCallbackFunction, this, _1);

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
		addMarker(MARKER_SEGMENTED_OBJECT,marker);
		addMarker(MARKER_ATTACHED_OBJECT,marker);
	}
}

//void SpherePickingRobotNavigator::run()
//{
//	ros::NodeHandle nh;
//	ros::AsyncSpinner spinner(4);
//	spinner.start();
//	srand(time(NULL));
//
//	ROS_INFO_STREAM(NODE_NAME<<" Setup stage started");
//	setup();
//	ROS_INFO_STREAM(NODE_NAME<<" Setup stage completed");
//
//	while(ros::ok())
//	{
//	    startCycleTimer();
//
//	    ROS_INFO_STREAM(NODE_NAME + ": Moving arm to side pose");
//		if(!moveArmToSide())
//		{
//			ROS_WARN_STREAM(NODE_NAME << ": Final side moved failed");
//			break;
//		}
//
//		ROS_INFO_STREAM(NODE_NAME + ": Segmentation stage started");
//		if(!performSegmentation())
//		{
//		  ROS_WARN_STREAM(NODE_NAME<<": Segmentation stage failed");
//		  continue;
//		}
//		ROS_INFO_STREAM(NODE_NAME << ": Segmentation stage completed");
//
//		ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage started");
//		if(!moveArmThroughPickSequence())
//		{
//		  ROS_WARN_STREAM(NODE_NAME << ": grasp pickup stage failed");
//		  continue;
//		}
//		else
//		{
//			ROS_INFO_STREAM(NODE_NAME << ": grasp pickup stage completed");
//		}
//
//		ROS_INFO_STREAM(NODE_NAME + ": grasp place stage started");
//		if(!moveArmThroughPlaceSequence())
//		{
//			ROS_WARN_STREAM(NODE_NAME << ": grasp place stage failed");
//		}
//		else
//		{
//			ROS_INFO_STREAM(NODE_NAME << ": grasp place stage completed");
//		}
//
//	    printTiming();
//	  }
//}

bool SpherePickingRobotNavigator::performSphereSegmentation()
{
	//  ===================================== preparing result objects =====================================
	  arm_navigation_msgs::CollisionObject obj;
	  int bestClusterIndex = -1;
	  sensor_msgs::PointCloud sphereCluster;

	  //  ===================================== calling segmentation =====================================
	  _SphereSegmentation.fetchParameters(SEGMENTATION_NAMESPACE);
	  bool success = _SphereSegmentation.segment(segmented_clusters_,obj,bestClusterIndex);

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
	  _SphereSegmentation.getSphereCluster(sphereCluster);

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
	  addDetectedObjectToLocalPlanningScene(obj);

	  //  ===================================== updating markers =====================================
	  // update markers
	  visualization_msgs::Marker &segMarker = getMarker(MARKER_SEGMENTED_OBJECT);
	  visualization_msgs::Marker &attachedMarker = getMarker(MARKER_ATTACHED_OBJECT);

	  // segmented object
	  collisionObjToMarker(obj,segMarker);
	  segMarker.action = visualization_msgs::Marker::ADD;
	  addMarker(MARKER_SEGMENTED_OBJECT,segMarker);

	  // attached object, hide for now until gripper is closed
	  collisionObjToMarker(obj,attachedMarker);
	  attachedMarker.action = visualization_msgs::Marker::DELETE;
	  attachedMarker.header.frame_id = gripper_link_name_;
	  addMarker(MARKER_ATTACHED_OBJECT,attachedMarker);

	  // ===================================== printing completion info message =====================================
	  arm_navigation_msgs::Shape &shape = obj.shapes[0];
	  ROS_INFO_STREAM("\n"<<NODE_NAME<<": Sphere Segmentation completed:\n");
	  ROS_INFO_STREAM("\tFrame id: "<<obj.header.frame_id<<"\n");
	  ROS_INFO_STREAM("\tRadius: "<<shape.dimensions[0]<<"\n");
	  ROS_INFO_STREAM("\tx: "<<pose.pose.position.x<<", y: "<<pose.pose.position.y
			  <<", z: "<<pose.pose.position.z<<"\n");

	return true;
}

bool SpherePickingRobotNavigator::performSegmentation()
{
	//RobotNavigator::performSegmentation();
	return RobotNavigator::performSegmentation() && performSphereSegmentation();
}

bool SpherePickingRobotNavigator::moveArmToSide()
{
	updateChangesToPlanningScene();
    _JointConfigurations.fetchParameters(JOINT_CONFIGURATIONS_NAMESPACE);
    return  moveArm(arm_group_name_,_JointConfigurations.SideAngles);

}

bool SpherePickingRobotNavigator::createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	ROS_INFO_STREAM(NODE_NAME<<": getting place position from ros parameters");
	_GoalParameters.fetchParameters(GOAL_NAMESPACE);
	_GoalParameters.generateNextLocationCandidates(placePoses);

	return true;
}

void SpherePickingRobotNavigator::GoalLocation::generateNextLocationCandidates(
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

void SpherePickingRobotNavigator::GoalLocation::generateNextLocationShuffleMode(
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

void SpherePickingRobotNavigator::GoalLocation::createCandidatePosesByRotation(const tf::Transform &startTrans,int numCandidates,tf::Vector3 axis,
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
