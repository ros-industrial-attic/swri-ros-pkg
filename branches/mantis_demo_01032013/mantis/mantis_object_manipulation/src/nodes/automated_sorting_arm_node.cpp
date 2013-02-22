/*
 * automated_sorting_arm_node.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: ros developer 
 */

#include <mantis_object_manipulation/arm_navigators/SortClutterArmNavigator.h>


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

		if(!moveArmToSide())
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

		if(!moveArmToSide())
		{
			return false;
		}

		while(proceed)
		{
			//segmentation in cluttered area for singulation
			clearResultsFromLastSrvCall();
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
				moveArmToSide();
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
				moveArmToSide();
				proceed = false;
				break;
			}

			// singulation completed, start moving part from singulated to sorted zone
			clearResultsFromLastSrvCall();
			if(!moveArmToSide())
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
				moveArmToSide();
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
				moveArmToSide();
				proceed = false;
				break;
			}
		}

		return proceed;
	}

	bool runClutterCycle()
	{

		bool proceed = true;

		if(!moveArmToSide())
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
				moveArmToSide();
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
				moveArmToSide();
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
