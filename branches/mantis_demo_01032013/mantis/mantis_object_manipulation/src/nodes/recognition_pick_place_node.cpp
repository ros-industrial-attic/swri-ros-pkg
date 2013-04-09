/*
 * recognition_pick_place_node.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: ros developer 
 */

#include <mantis_object_manipulation/arm_navigators/SortClutterArmNavigator.h>
#include <iostream>

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
		ros::AsyncSpinner spinner(4);
		spinner.start();
		srand(time(NULL));

		// arm setup
		setup();

		// cycle
		while(ros::ok())
		{
			moveArmHome();

			std::cout<<"\n\tPress enter to proceed: _";
			std::cin.get(); // should lock the terminal until enter is pressed

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
			if(performPickGraspPlanning() && performPlaceGraspPlanning(singulated_dropoff_location_))
			{
				ROS_INFO_STREAM("Grasp planning succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Grasp planning failed");
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

			// moving for place in singulated zone
			ROS_INFO_STREAM("Place started");
			if(moveArmThroughPlaceSequence())
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
		AutomatedPickerRobotNavigator::setup();
	}

	virtual void fetchParameters(std::string name_space = "")
	{
		AutomatedPickerRobotNavigator::fetchParameters(nameSpace);
		singulated_dropoff_location_.fetchParameters(singulated_dropoff_ns_);
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
		grasp_candidates_[0].grasp_pose = candidate_pick_poses_[0].pose;
		//manipulation_utils::generateCandidateGrasps(firstGrasp,tf::Vector3(0.0f,0.0f,1.0f),8,grasp_candidates_);

		// instantiating needed transforms and poses
		tf::StampedTransform wrist_in_tcp_tf = tf::StampedTransform();
		tf::Transform object_in_world_tf; // will remove
		tf::Transform object_in_world_inverse_tf;// will remove

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
//		grasp_pickup_goal_.arm_name = arm_group_name_;
		grasp_pickup_goal_.collision_object_name = modelId;
//		grasp_pickup_goal_.lift.direction.header.frame_id = cm_.getWorldFrameId();
//		grasp_pickup_goal_.lift.direction.vector.z = 1.0;
//		grasp_pickup_goal_.lift.desired_distance = .1;
		grasp_pickup_goal_.target.reference_frame_id = modelId;
//		grasp_pickup_goal_.target.cluster = segmented_clusters_[0];
		grasp_pickup_goal_.allow_gripper_support_collision = true;
//		grasp_pickup_goal_.collision_support_surface_name = "table";
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

	virtual bool moveArmHome()
	{
		return updateChangesToPlanningScene() && moveArm(arm_group_name_,joint_home_conf_.SideAngles);
	}

protected:

	JointConfiguration joint_home_conf_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"recognition_pick_place_node");

	RobotNavigator::Ptr navigator_ptr = RobotNavigator::Ptr(new RecognitionPickNavigator());
	navigator_ptr->run();

	return 0;
}
