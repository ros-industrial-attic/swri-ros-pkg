/*
 * automated_sorting_arm_node.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: ros developer 
 */

#include <mantis_object_manipulation/arm_navigators/SortClutterArmNavigator.h>

static const std::string JOINT_WAIT_POSITION = "joint_wait_position";
static const std::string JOINT_HOME_POSITION = "joint_home_position";

class SortingRobotNavigator: public SortClutterArmNavigator
{
public:
	SortingRobotNavigator()
	{

	}

	~SortingRobotNavigator()
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

		if(!moveArmHome())
		{
			ROS_WARN_STREAM(NODE_NAME << ": Side moved failed");
			return;
		}
		ROS_INFO_STREAM("Arm at home position");

		// sort/clutter cycle
		while(ros::ok())
		{
			ROS_INFO_STREAM("Sorting cycle started");
			if(runSortCycle() )
			{
				ROS_INFO_STREAM("Sorting cycle completed");
			}
			else
			{
				ROS_ERROR_STREAM("Sorting cycle failed, exiting");
				break;
			}

			ROS_INFO_STREAM("Clutter cycle started");
			if(runClutterCycle())
			{
				ROS_INFO_STREAM("Clutter cycle completed");
			}
			else
			{
				ROS_ERROR_STREAM("Clutter cycle failed, exiting");
				break;
			}
		}

		spinner.stop();
		ros::shutdown();
	}

	bool runSortCycle()
	{
		// moving parts from cluttered to sorted zone
		bool proceed = true;

		if(!moveArmHome())
		{
			return false;
		}

		while(proceed)
		{

			clearResultsFromLastSrvCall();

			// moving away from camera view
			ROS_INFO_STREAM("Moving to wait position");
			if(moveArmToSide())
			{
				ROS_INFO_STREAM("Move to wait completed");
			}
			else
			{
				ROS_WARN_STREAM("Move to wait failed");
				moveArmHome();
				proceed = false;
				break;
			}

			//segmentation in cluttered area for singulation
			zone_selector_.goToPickZone(cluttered_zone_index_);
			ROS_INFO_STREAM("Segmentation stage started");
			if(performSegmentation())
			{
				ROS_INFO_STREAM("Segmentation stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Segmentation stage failed");
				break;
			}

			// planning for singulation
			ROS_INFO_STREAM("Grasp Planning stage started");
			if(performGraspPlanningForSingulation())
			{
				ROS_INFO_STREAM("Grasp Planning stage completed");
			}
			else
			{
				ROS_ERROR_STREAM("Grasp Planning stage failed");
				break;
			}

			// moving for pickup in cluttered zone
			ROS_INFO_STREAM("Grasp Pick stage started");
			if(moveArmThroughPickSequence())
			{
				ROS_INFO_STREAM("Grasp Pick stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Grasp Pick stage failed");
				moveArmHome();
				proceed = false;
				break;
			}

			// moving for place in singulated zone
			ROS_INFO_STREAM("Grasp Place stage started");
			if(moveArmThroughPlaceSequence())
			{
				ROS_INFO_STREAM("Grasp Place stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Grasp Place stage failed");
				moveArmHome();
				proceed = false;
				break;
			}

			// singulation completed, start moving part from singulated to sorted zone
			clearResultsFromLastSrvCall();
			if(!moveArmHome())
			{
				proceed = false;
				break;
			}

			// perception for sorting
			zone_selector_.goToPickZone(singulation_zone_index_);
			ROS_INFO_STREAM("Segmentation stage started");
			if(performSegmentation())
			{
				ROS_INFO_STREAM("Segmentation stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Segmentation stage failed");
				break;
			}

			ROS_INFO_STREAM("Recognition stage started");
			if(performRecognition())
			{
				ROS_INFO_STREAM("Recognition stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Recognition stage failed");
				break;
			}

			// planning in singulated zone
			ROS_INFO_STREAM("Grasp Planning stage started");
			if(performGraspPlanningForSorting())
			{
				ROS_INFO_STREAM("Grasp Planning stage completed");
			}
			else
			{
				ROS_ERROR_STREAM("Grasp Planning stage failed");
				break;
			}

			// moving for pickup in singulated zone
			ROS_INFO_STREAM("Grasp Pick stage started");
			if(moveArmThroughPickSequence())
			{
				ROS_INFO_STREAM("Grasp Pick stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Grasp Pick stage failed");
				moveArmHome();
				proceed = false;
				break;
			}

			// moving for place in sorted zone
			ROS_INFO_STREAM("Grasp Place stage started");
			if(moveArmThroughPlaceSequence())
			{
				ROS_INFO_STREAM("Grasp Place stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Grasp Place stage failed");
				moveArmHome();
				proceed = false;
				break;
			}

		}

		return proceed;
	}

	bool runClutterCycle()
	{

		bool proceed = true;

		if(!moveArmHome())
		{
			return false;
		}

		while(proceed)
		{

			// perception , moving from sorted to cluttered
			clearResultsFromLastSrvCall();
			zone_selector_.goToPickZone(sorted_zone_index_);
			ROS_INFO_STREAM("Segmentation stage started");
			if(performSegmentation())
			{
				ROS_INFO_STREAM("Segmentation stage completed");
			}
			else
			{
				ROS_WARN_STREAM(": Segmentation stage failed");
				break;
			}

			// planning
			ROS_INFO_STREAM(": Grasp Planning stage started");
			if(performGraspPlanningForClutter())
			{
				ROS_INFO_STREAM(": Grasp Planning stage completed");
			}
			else
			{
				ROS_ERROR_STREAM("Grasp Planning stage failed");
				break;
			}

			// moving for pickup in sorted zone
			ROS_INFO_STREAM("Grasp Pick stage started");
			if(moveArmThroughPickSequence())
			{
				ROS_INFO_STREAM("Grasp Pick stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Grasp Pick stage failed");
				moveArmHome();
				proceed = false;
				break;
			}

			// moving for place in clutter zone
			ROS_INFO_STREAM("Grasp Place stage started");
			if(moveArmThroughPlaceSequence())
			{
				ROS_INFO_STREAM("Grasp Place stage completed");
			}
			else
			{
				ROS_WARN_STREAM("Grasp Place stage failed");
				moveArmHome();
				proceed = false;
				break;
			}

		}

		return proceed;
	}

protected:

	virtual void setup()
	{
		AutomatedPickerRobotNavigator::setup();
	}

	virtual void fetchParameters(std::string name_space = "")
	{
		SortClutterArmNavigator::fetchParameters(name_space);
		joint_home_conf.fetchParameters(NODE_NAME + "/" + JOINT_HOME_POSITION);
		joint_wait_conf.fetchParameters(NODE_NAME + "/" + JOINT_WAIT_POSITION);

	}

	virtual bool moveArmToSide()
	{
	    return updateChangesToPlanningScene() && moveArm(arm_group_name_,joint_wait_conf.SideAngles);
	}

	virtual bool moveArmHome()
	{
		return updateChangesToPlanningScene() && moveArm(arm_group_name_,joint_home_conf.SideAngles);
	}

	JointConfiguration joint_home_conf;
	JointConfiguration joint_wait_conf;

};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"automated_sorting_arm_node");
	ros::NodeHandle nh;

	ROS_INFO_STREAM(ros::this_node::getName()<<": Creating sorting arm navigator");
	RobotNavigator::Ptr navigator(new SortingRobotNavigator());

	ROS_INFO_STREAM(ros::this_node::getName()<<": Sort Arm Navigator started");
	navigator->run();
	return 0;
}
