/*
 * AutomatedPickerRobotNavigator.h
 *
 *  Created on: Nov 6, 2012
 *      Author: coky
 */

#ifndef AUTOMATEDPICKERROBOTNAVIGATOR_H_
#define AUTOMATEDPICKERROBOTNAVIGATOR_H_

#include <object_manipulation_tools/robot_navigators/RobotNavigator.h>
#include <perception_tools/segmentation/SphereSegmentation.h>
#include <mantis_object_manipulation/zone_selection/PickPlaceZoneSelector.h>
#include <boost/thread/mutex.hpp>

static const std::string PARAM_NAME_NUM_GRASP_ATTEMTPTS = "num_of_grasp_attempts";
static const std::string PARAM_NAME_NEW_GRASP_OFFSET = "new_grasp_offset";
static const std::string PARAM_NAME_ATTACHED_OBJECT_BB_SIDE = "attached_object_bb_side";

class AutomatedPickerRobotNavigator: public RobotNavigator
{
public:

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

	AutomatedPickerRobotNavigator();
	virtual ~AutomatedPickerRobotNavigator();
	//virtual void run();

	static std::string MARKER_SEGMENTED_OBJECT;
	static std::string SEGMENTATION_NAMESPACE;
	static std::string GOAL_NAMESPACE;
	static std::string JOINT_CONFIGURATIONS_NAMESPACE;
	static std::string MARKER_ARRAY_TOPIC;

protected:

	virtual void setup();
	virtual void fetchParameters(std::string nameSpace = "");

	virtual bool performSegmentation();
	bool performSphereSegmentation();
	virtual bool performRecognition();
	virtual bool performGraspPlanning();

	virtual bool createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses);
	virtual bool moveArmToSide();
	virtual bool moveArmThroughPickSequence();
	virtual bool moveArmThroughPlaceSequence();
	// callback overrides
	virtual void callbackPublishMarkers(const ros::TimerEvent &evnt);

	// helper methods
	void updateMarkerArrayMsg();
	void generateGraspPoses(const geometry_msgs::Pose &pose,int numCandidates,
		std::vector<geometry_msgs::Pose> &poses); // generates extra grasp poses by rotating about the approach vector




protected:

	// ros parameters
	PickPlaceZoneSelector zone_selector_;
	JointConfiguration joint_configuration_;
	int num_of_grasp_attempts_; // number of additional pick attempts
	double offset_from_first_grasp_;// distance from original pick grasp to used in new pick attempt
	double attached_obj_bb_side_;

	// segmentation
	SphereSegmentation sphere_segmentation_;

	//services
	ros::ServiceClient recognition_client_;

	// recognition results
	arm_navigation_msgs::CollisionObject recognized_collision_object_;
	int recognized_obj_id_;

	// candidate pick poses to evaluate
	std::vector<geometry_msgs::PoseStamped> candidate_pick_poses_;

	// place poses
	std::vector<geometry_msgs::PoseStamped> candidate_place_poses_;

	// publishers
	ros::Publisher marker_array_pub_;

	// visualization
	visualization_msgs::MarkerArray marker_array_msg_;

	// threading
	boost::mutex marker_array_mutex_;

};

#endif /* AUTOMATEDPICKERROBOTNAVIGATOR_H_ */
