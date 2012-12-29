/*
 * dual_arm_supervisor_node.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: jnicho
 */

#include <ros/ros.h>
#include <mantis_object_manipulation/ArmHandshaking.h>
#include <limits>

const std::string ARM1_HANDSHAKING_SERVICE_NAME = "arm1_handshaking_service";
const std::string ARM2_HANDSHAKING_SERVICE_NAME = "arm2_handshaking_service";
const int MAX_SERVICE_CALL_ATTEMPTS = 8;

typedef mantis_object_manipulation::ArmHandshaking ArmServiceType;
typedef mantis_object_manipulation::ArmHandshaking::Response ArmResponse;
typedef mantis_object_manipulation::ArmHandshaking::Request ArmRequest;

// ros parameters
static const std::string PARAM_MAX_CYCLE_COUNT="max_cycle_count";
static const std::string PARAM_INTERRUPT_CYCLE="interrupt_cycle";
static const std::string PARAM_CONTINUE_ON_CYCLE_FAIL="continue_on_cycle_fail";

class DualArmSupervisor
{

public:
	struct TaskDetails
	{
	public:
		TaskDetails(std::string name,ros::ServiceClient &arm_client, uint32_t command):
			name_(name),
			arm_client_(arm_client),
			command_(command)
		{

		}

	public:
		std::string name_;
		ros::ServiceClient arm_client_;
		uint32_t command_;
	};

public:
	DualArmSupervisor():
		interrupt_cycle_(false),
		continue_on_cycle_fail_(false),
		max_cycle_count_(std::numeric_limits<int>::infinity())
	{

	}

	~DualArmSupervisor()
	{

	}

	void run()
	{
		if(!setup())
		{
			return;
		}

		// designating clients for clutter and sort
		ros::ServiceClient &clutter_arm_client_ = arm1_handshaking_client_;
		ros::ServiceClient &sorting_arm_client_ = arm2_handshaking_client_;

		// creating task sequences
		std::vector<TaskDetails> clutter_to_sort_seq, sort_to_clutter_seq;

		clutter_to_sort_seq.push_back(TaskDetails("Singulate Cluster",clutter_arm_client_,ArmRequest::SINGULATE_CLUTTER));
		clutter_to_sort_seq.push_back(TaskDetails("Sort",sorting_arm_client_,ArmRequest::SORT));

		sort_to_clutter_seq.push_back(TaskDetails("Singulate Sorted",sorting_arm_client_,ArmRequest::SINGULATE_SORTED));
		sort_to_clutter_seq.push_back(TaskDetails("Clutter",clutter_arm_client_,ArmRequest::CLUTTER));


		while(ros::ok() &&
				(cycleThroughTaskSequence(clutter_to_sort_seq) || continue_on_cycle_fail_)&&
				(cycleThroughTaskSequence(sort_to_clutter_seq)|| continue_on_cycle_fail_))
		{
			ROS_INFO_STREAM(node_name_<<": Clutter to Sorted to Clutter cycle finished");
		}
	}

protected:

	// members strings
	std::string node_name_;

	// service clients
	ros::ServiceClient arm1_handshaking_client_;
	ros::ServiceClient arm2_handshaking_client_;

	// ros parameters
	int max_cycle_count_;
	bool interrupt_cycle_;
	bool continue_on_cycle_fail_;

protected:

	bool fetchParameters()
	{
		ros::NodeHandle nh("~");
		return nh.getParam(PARAM_MAX_CYCLE_COUNT,max_cycle_count_) &&
				nh.getParam(PARAM_INTERRUPT_CYCLE,interrupt_cycle_) &&
				nh.getParam(PARAM_CONTINUE_ON_CYCLE_FAIL,continue_on_cycle_fail_);
	}

	bool checkServiceClientConnections()
	{
		return (arm1_handshaking_client_ != NULL) && (arm2_handshaking_client_ != NULL);
	}

	bool setup()
	{
		ros::NodeHandle nh;
		node_name_ = ros::this_node::getName();

		// getting parameters
		if(fetchParameters())
		{
			ROS_INFO_STREAM(ros::this_node::getName()<<": Found parameters");
		}
		else
		{
			ROS_ERROR_STREAM(ros::this_node::getName()<<": Did not find parameters, proceeding anyway");
		}

		// setting up clients
		if(ros::service::waitForService(ARM1_HANDSHAKING_SERVICE_NAME,-1) &&
				ros::service::waitForService(ARM2_HANDSHAKING_SERVICE_NAME,-1))
		{
			arm1_handshaking_client_ = nh.serviceClient<mantis_object_manipulation::ArmHandshaking>(
				ARM1_HANDSHAKING_SERVICE_NAME,true);
			arm2_handshaking_client_ = nh.serviceClient<mantis_object_manipulation::ArmHandshaking>(
					ARM2_HANDSHAKING_SERVICE_NAME,true);
			ROS_INFO_STREAM(node_name_<<": Successfully connected to arm handshaking services");
		}
		else
		{
			ROS_ERROR_STREAM(node_name_<<": Could not establish connection to handshaking services ");
			return false;
		}

		// sending both robots to the home position
		ArmServiceType armSrv;
		armSrv.request.command = ArmRequest::MOVE_HOME;
		if(!arm1_handshaking_client_.call(armSrv) ||  !arm2_handshaking_client_.call(armSrv))
		{
			ROS_ERROR_STREAM(node_name_<<" Could not move arms to home position, exiting");
			return false;
		}

		return true;
	}

	/*	This sends command and checks that it completed
	 *
	 */
	bool sendAndMonitorTask(ros::ServiceClient &client_arm, bool &early_exit,unsigned int command)
	{
		ArmRequest req;
		ArmResponse res;
		int srv_call_counter = 1;
		bool proceed = false;

		early_exit = false;
		req.command = command;
		while((srv_call_counter < MAX_SERVICE_CALL_ATTEMPTS) && (!proceed) && client_arm.call(req,res))
		{
			srv_call_counter++;
			proceed = (bool)res.completed;
			if(!proceed)
			{
				switch(res.error_code)
				{
				case ArmResponse::NO_CLUSTERS_FOUND:
					// no clusters in clutter pick zone, ok condition but early exit
					ROS_WARN_STREAM(node_name_<<": arm reported zero clusters, exiting early");
					early_exit = true;
					proceed = true;
					break;

				case ArmResponse::PERCEPTION_ERROR:
				case ArmResponse::PROCESSING_ERROR:
				case ArmResponse::GRASP_PLANNING_ERROR:
				case ArmResponse::PLACE_ERROR:

					// will try again
					proceed = false;
					req.command = command;
					break;

				case ArmResponse::FINAL_MOVE_HOME_ERROR:

					ROS_ERROR_STREAM(node_name_<<": Arm did not return home, requesting move home");
					req.command = ArmRequest::MOVE_HOME;
					proceed = false;
					break;

				default:

					ROS_ERROR_STREAM(node_name_<<"Received unknown error code from arm");
					return false;
				}
			}
		}

		return proceed;
	}

	bool cycleThroughTaskSequence(std::vector<TaskDetails> task_seq)
	{
		bool early_termination = false; // indicates that all objects have been handled
		int current_index = 0;
		int cycle_count = 0;
		while(!early_termination)
		{
			fetchParameters();
			if(++cycle_count > (max_cycle_count_ < 0 ? std::numeric_limits<int>::infinity() : max_cycle_count_)
					|| interrupt_cycle_ )
			{
				ros::NodeHandle nh("~");
				interrupt_cycle_ = false;
				nh.setParam(PARAM_INTERRUPT_CYCLE,interrupt_cycle_);
				break;
			}

			TaskDetails &task = task_seq[current_index];
			ROS_INFO_STREAM(node_name_<<": "<< task.name_<<" requested");
			if(checkServiceClientConnections() && sendAndMonitorTask(task.arm_client_,early_termination,task.command_))
			{
				current_index = (++current_index < task_seq.size()) ? current_index : 0 ;
			}
			else
			{
				ROS_INFO_STREAM(node_name_<<": "<< task.name_<<" failed, terminating");
				return false;
			}
			ROS_INFO_STREAM(node_name_<<": "<< task.name_<<" completed");
		}

		return true;
	}
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"dual_arm_supervisor");
	ros::NodeHandle nh;

	DualArmSupervisor arm_supervisor;
	arm_supervisor.run();

	return 0;
}
