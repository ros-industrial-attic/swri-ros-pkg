/*
 * AutomatedPickerRobotNavigator.cpp
 *
 *  Created on: Nov 6, 2012
 *      Author: coky
 */

//#define USE_SPHERE_SEGMENTATION

#include <mantis_object_manipulation/arm_navigators/AutomatedPickerRobotNavigator.h>
#include <mantis_object_manipulation/arm_navigators/AutomatedPickerRobotNavigator.h>
#include <mantis_perception/mantis_recognition.h>

std::string AutomatedPickerRobotNavigator::MARKER_SEGMENTED_OBJECT = "segmented_obj";
std::string AutomatedPickerRobotNavigator::SEGMENTATION_NAMESPACE = "segmentation";
std::string AutomatedPickerRobotNavigator::GOAL_NAMESPACE = "goal";
std::string AutomatedPickerRobotNavigator::JOINT_CONFIGURATIONS_NAMESPACE = "joints";
std::string AutomatedPickerRobotNavigator::MARKER_ARRAY_TOPIC = "object_array";

// global variables
const double OBJECT_ABB_SIDE = 0.1; // this variable will be used to set the bounds of each object perceived.  Eventually, the recognition
									// service will provide this value and this variable will be removed.

AutomatedPickerRobotNavigator::AutomatedPickerRobotNavigator()
:RobotNavigator()
{
	// TODO Auto-generated constructor stub
	GOAL_NAMESPACE = NODE_NAME + "/" + GOAL_NAMESPACE;
	SEGMENTATION_NAMESPACE = NODE_NAME + "/" + SEGMENTATION_NAMESPACE;
	JOINT_CONFIGURATIONS_NAMESPACE = NODE_NAME + "/" + JOINT_CONFIGURATIONS_NAMESPACE;

}

AutomatedPickerRobotNavigator::~AutomatedPickerRobotNavigator()
{
	// TODO Auto-generated destructor stub
	ROS_INFO_STREAM(NODE_NAME<<": Exiting navigator");
}

void AutomatedPickerRobotNavigator::setup()
{
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();

	ROS_INFO_STREAM(NODE_NAME<<": Loading ros parameters");

	// getting ros parametets
	fetchParameters(NAVIGATOR_NAMESPACE);
	zone_selector_.fetchParameters(NODE_NAME);

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

		//recognition
		//recognition_client_ = nh.serviceClient<mantis_perception::mantis_recognition>("/mantis_object_recognition");
		recognition_client_ = nh.serviceClient<mantis_perception::mantis_recognition>(recognition_service_,true);

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
		marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC,1);
		attached_object_publisher_ = nh.advertise<arm_navigation_msgs::AttachedCollisionObject> ("attached_collision_object_alternate", 1);

		// setting up timer obj
		marker_pub_timer_ = nh.createTimer(ros::Duration(0.4f),&AutomatedPickerRobotNavigator::callbackPublishMarkers,this);

		ROS_INFO_STREAM(NODE_NAME<<": Setting up dynamic libraries");

		// others
		grasp_tester_ = GraspTesterPtr(new object_manipulator::GraspTesterFast(&cm_, ik_plugin_name_));
		place_tester_ = PlaceSequencePtr(new PlaceSequenceValidator(&cm_, ik_plugin_name_));
		trajectories_finished_function_ = boost::bind(&AutomatedPickerRobotNavigator::trajectoriesFinishedCallbackFunction, this, _1);

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

		// pick and place zone markers
		updateMarkerArrayMsg();
	}
}

bool AutomatedPickerRobotNavigator::performSphereSegmentation()
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
	  recognized_collision_object_ = obj;

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

bool AutomatedPickerRobotNavigator::performRecognition()
{
	// declaring temporary variables to generate object ids
#ifdef USE_SPHERE_SEGMENTATION

	static const int MaxIdCount = 8;
	static int CurrentIdCount = 1;

	ROS_WARN_STREAM(NODE_NAME<<": Using id "<<CurrentIdCount<<" for the next object");

	// assigning id ( if recognition is used the id returned in the result should be used instead)
	recognized_obj_id_ = CurrentIdCount;
	CurrentIdCount++;
	if(CurrentIdCount > MaxIdCount)
	{
		CurrentIdCount = 1;
	}

	// passed recognized object details to zone selector
	double bbBoxSide = zone_selector_.getPlaceZone().MinObjectSpacing;
	tf::Vector3 objSize = tf::Vector3(bbBoxSide,bbBoxSide,bbBoxSide);
	PickPlaceZoneSelector::ObjectDetails objDetails(tf::Transform::getIdentity(),objSize,
			CurrentIdCount,"next_object");
	zone_selector_.getPlaceZone().setNextObjectDetails(objDetails);

	// computing poses so that no move is attempted if no locations are available in the place zone.
	candidate_place_poses_.clear();
	if(!zone_selector_.generateNextLocationCandidates(candidate_place_poses_))
	{
		ROS_WARN_STREAM(NODE_NAME<<": Couldn't find available location for object, swapping zones and resetting counter.");
		// no more locations available, swapping zones
		zone_selector_.swapPickPlaceZones();
		CurrentIdCount = 1;
		return false;
	}

#else

	// preparing recognition results
	arm_navigation_msgs::CollisionObject obj;
	obj.id = makeCollisionObjectNameFromModelId(0);
	mantis_perception::mantis_recognition rec_srv;
	rec_srv.request.clusters = segmented_clusters_;
	rec_srv.request.table = segmentation_results_.table;

	// recognition call
	if (!recognition_client_.call(rec_srv))
	{
	  ROS_ERROR("Call to mantis recognition service failed");
	  return false;
	}
	else
	{
		ROS_WARN_STREAM(NODE_NAME<<": Found object with id: "<<rec_srv.response.model_id);
	}

/*
 * Storing recognition results
 */
	// parsing pose results
	tf::Transform t = tf::Transform::getIdentity();
	nrg_object_recognition::pose &tempPose =  rec_srv.response.pose;
	t.setOrigin(tf::Vector3(tempPose.x,tempPose.y,tempPose.z));
	t.setRotation(tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),tempPose.rotation));

	// storing pose
	geometry_msgs::PoseStamped pose;
	tf::poseTFToMsg(t,pose.pose);
	pose.header.frame_id = cm_.getWorldFrameId();
	recognized_obj_pose_map_[std::string(obj.id)] = pose;

	// adding collision obj to planning scene
	obj.header.frame_id = cm_.getWorldFrameId();
	obj.padding = 0.0f;
	obj.shapes = std::vector<arm_navigation_msgs::Shape>();
	arm_navigation_msgs::Shape shape;
	shape.type = arm_navigation_msgs::Shape::SPHERE;
	//shape.dimensions.push_back(zone_selector_.getPlaceZone().MinObjectSpacing/2.0f); // radius;
	shape.dimensions.push_back(0.02f);
	obj.shapes.push_back(shape);
	obj.poses.push_back(pose.pose);
	addDetectedObjectToLocalPlanningScene(obj);

	// storing model detail for path planning
	household_objects_database_msgs::DatabaseModelPoseList models;
	household_objects_database_msgs::DatabaseModelPose model;
	model.model_id = 0;
	model.pose = pose;
	model.confidence = 1.0f;
	model.detector_name = "cfh_recognition";
	models.model_list.push_back(model);
	recognized_models_.clear();
	recognized_models_.push_back(models);
	//recognized_model_description_.name = segmentation_results_.table.pose.header.frame_id;

/*
 * Finished storing recognition results
 */

	// passed recognized object details to zone selector
	double bbBoxSide = OBJECT_ABB_SIDE;
	tf::Vector3 objSize = tf::Vector3(bbBoxSide,bbBoxSide,bbBoxSide);
	PickPlaceZoneSelector::ObjectDetails objDetails(tf::Transform::getIdentity(),objSize,
			rec_srv.response.model_id,rec_srv.response.label);
	zone_selector_.setNextObjectDetails(objDetails);

	// computing poses so that no move is attempted if no locations are available in the place zone.
	candidate_place_poses_.clear();

	ROS_WARN_STREAM(NODE_NAME<<": using box with size "<<OBJECT_ABB_SIDE);
	if(!zone_selector_.generateNextLocationCandidates(candidate_place_poses_))
	{
		ROS_WARN_STREAM(NODE_NAME<<": Couldn't find available location for object, swapping zones.");
		// no more locations available, swapping zones
		zone_selector_.goToNextPickZone();
		updateMarkerArrayMsg();
		//CurrentIdCount = 1;
		return false;
	}

#endif

	return true;
}

bool AutomatedPickerRobotNavigator::performSegmentation()
{
	bool success = false;
	success =  RobotNavigator::performSegmentation();
	if(!success)
	{
		return false;
	}

	// check if at least one cluster is located in pick zone
	std::vector<int> inZone;
	success = zone_selector_.isInPickZone(segmented_clusters_,inZone);
	if(!success)
	{
		ROS_WARN_STREAM(NODE_NAME<<": Neither cluster was found in pick zone, swapping zones");
		zone_selector_.goToNextPickZone();
		updateMarkerArrayMsg();
		return false;
	}
	else
	{
		ROS_INFO_STREAM(NODE_NAME<<": A total of "<<inZone.size()<<" were found in pick zone");

		// retaining only cluster in pick zone
		std::vector<sensor_msgs::PointCloud> tempArray;
		for(unsigned int i = 0;i < inZone.size();i++)
		{
			tempArray.push_back(segmented_clusters_[inZone[i]]);
		}
		segmented_clusters_.assign(tempArray.begin(),tempArray.end());
	}

#ifdef USE_SPHERE_SEGMENTATION
	success =  performSphereSegmentation();
	if(!success)
	{
		return false;
	}

#endif
	return true;

}

bool AutomatedPickerRobotNavigator::moveArmToSide()
{

    _JointConfigurations.fetchParameters(JOINT_CONFIGURATIONS_NAMESPACE);
    return updateChangesToPlanningScene() && moveArm(arm_group_name_,_JointConfigurations.SideAngles);
}

bool AutomatedPickerRobotNavigator::createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses)
{
	// will copy previously computed candidate poses
	placePoses.assign(candidate_place_poses_.begin(),candidate_place_poses_.end());
	return true;//zone_selector_.generateNextLocationCandidates(placePoses);
}

void AutomatedPickerRobotNavigator::callbackPublishMarkers(const ros::TimerEvent &evnt)
{
	RobotNavigator::callbackPublishMarkers(evnt);
	//marker_array_pub_.publish(marker_array_msg_);

	boost::mutex::scoped_lock lock(marker_array_mutex_);
	{
		if(!marker_array_msg_.markers.empty())
		{
			marker_array_pub_.publish(marker_array_msg_);
		}
	}
}

void AutomatedPickerRobotNavigator::updateMarkerArrayMsg()
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
		zone_selector_.getAllActiveZonesMarkers(marker_array_msg_);
	}
}



