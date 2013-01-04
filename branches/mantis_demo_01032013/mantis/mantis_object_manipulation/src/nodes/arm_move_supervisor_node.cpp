/*
 * dual_arm_supervisor_node.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: jnicho
 */

#include <ros/ros.h>
#include <mantis_object_manipulation/ArmHandshaking.h>
#include <boost/thread.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/assign/list_of.hpp>
#include <limits>

const std::string ARM1_HANDSHAKING_SERVICE_NAME = "arm1_handshaking_service";
const std::string ARM2_HANDSHAKING_SERVICE_NAME = "arm2_handshaking_service";
const int MAX_SERVICE_CALL_ATTEMPTS = 4;

typedef mantis_object_manipulation::ArmHandshaking ArmServiceType;
typedef mantis_object_manipulation::ArmHandshaking::Response ArmResponse;
typedef mantis_object_manipulation::ArmHandshaking::Request ArmRequest;

// ros parameters
static const std::string PARAM_MAX_CYCLE_COUNT="max_cycle_count";
static const std::string PARAM_INTERRUPT_CYCLE="interrupt_cycle";
static const std::string PARAM_CONTINUE_ON_CYCLE_FAIL="continue_on_cycle_fail";

class ConcurrentArmMoveSupervisor
{

public:
	struct TaskDetails
	{
	public:
		TaskDetails()
		{

		}

		TaskDetails(std::string name,ros::ServiceClient &arm_client,std::vector<uint32_t> &task_codes,
				uint32_t attempts = 1,uint32_t termination_error_code = 0,uint32_t recovery_error_required = 0)
		:	name_(name),
			arm_client_(arm_client),
			tasks_codes_(task_codes),
			attempts_(attempts),
			termination_error_code_(termination_error_code),
			recovery_valid_error_code_(recovery_error_required),
			completed_(true),
			error_code_(ArmResponse::OK),
			error_code_description_("OK"),
			recovery_task_termination_error_(false)
		{

		}

	public:
		std::string name_;
		ros::ServiceClient arm_client_;

		// request members
		//uint32_t command_code_; // simpler higher level command code
		std::vector<uint32_t> tasks_codes_;
		uint32_t attempts_;

		// response members
		bool completed_;
		uint32_t error_code_;
		std::string error_code_description_;

		// error handling members
		uint32_t termination_error_code_; // if this error code was sent then terminate application.
		bool recovery_task_termination_error_;

		// error recovery members
		uint32_t recovery_valid_error_code_;// used 0nly if task is assigned to a recovery task set
		std::vector<std::pair<bool,TaskDetails> > recovery_tasks_bool_pair_;

	public:

		bool terminationErrorReceived()
		{
			return termination_error_code_ == error_code_ || recovery_task_termination_error_;
		}

		void addRecoveryTask(bool check_error_code_first, TaskDetails &task)
		{
			recovery_tasks_bool_pair_.push_back(std::make_pair(check_error_code_first,task));
		}

		bool getRecoveryTasks(std::vector<TaskDetails> &recovery_tasks)
		{
			bool tasks_found = false;
			std::vector<std::pair<bool,TaskDetails> >::iterator i;
			bool check_error_code;
			uint32_t required_error_code;
			for(i = recovery_tasks_bool_pair_.begin(); i != recovery_tasks_bool_pair_.end(); i++)
			{
				check_error_code = i->first;
				required_error_code = i->second.recovery_valid_error_code_;
				// setting service client
				i->second.arm_client_ = arm_client_;
				if(check_error_code && (required_error_code == error_code_))
				{
					recovery_tasks.push_back(i->second);
					tasks_found = true;
				}

				if(!check_error_code)
				{
					recovery_tasks.push_back(i->second);
					tasks_found = true;
				}
			}

			return tasks_found;
		}

		bool sendAndMonitorTask()
		{
			ArmRequest req;
			ArmResponse res;
			uint32_t srv_call_counter = 0;
			bool succeeded = false;
			std::vector<TaskDetails> recovery_tasks;

			req.tasks_codes = tasks_codes_;
			while((srv_call_counter++ < attempts_) && !succeeded)
			{
				ROS_INFO_STREAM(ros::this_node::getName()<<": Task '"<<name_<<"' attempt "<<srv_call_counter);
				succeeded = arm_client_.call(req,res) && (bool)res.completed;
				ROS_INFO_STREAM(ros::this_node::getName()<<": Task '"<<name_<<"' state "
						<<(succeeded ?" succeeded":" failed"));
			}
			error_code_ = res.error_code;

			// on failure execute recovery task serially
			if(!succeeded && getRecoveryTasks(recovery_tasks))
			{
				std::vector<TaskDetails>::iterator i;
				for(i = recovery_tasks.begin(); i != recovery_tasks.end(); i++)
				{
					if(!i->sendAndMonitorTask() )
					{
						// checking termination error, and exiting
						recovery_task_termination_error_ = i->terminationErrorReceived();
						break;
					}
				}
			}
			completed_ = succeeded;
			return completed_;
		}
	};

public:
	typedef std::vector<TaskDetails> ConcurrentTaskSet;
	typedef std::vector<ConcurrentTaskSet> TaskSequence;

	class TaskSetExecutor
	{
	public:

		TaskSetExecutor()
		{

		}

		virtual ~TaskSetExecutor()
		{

		}

		/*
		 * runs all tasks in a multithreaded fashion.  Returns false if either task wasn't completed,
		 * returns true otherwise
		 */
		bool runTaskSet(ConcurrentTaskSet &task_set)
		{

			boost::thread_group task_thread_group;

			ConcurrentTaskSet::iterator i;
			for(i = task_set.begin(); i != task_set.end(); i++)
			{
				TaskDetails &task = *i;
				task_thread_group.create_thread(boost::bind(&TaskDetails::sendAndMonitorTask,&task));
			}

			// waiting for all tasks to complete
			task_thread_group.join_all();

			// collecting completion flags
			bool success = true;
			for(i = task_set.begin(); success && i != task_set.end(); i++)
			{
				success= success && i->completed_;
			}
			return success;
		}

		bool runTaskSequence(TaskSequence &task_seq,bool &termination_error_found)
		{
			bool completed = true;
			TaskSequence::iterator i;

			for(i = task_seq.begin(); i != task_seq.end(); i++)
			{
				ROS_INFO_STREAM(ros::this_node::getName()<<": Requesting Multithreaded Task(s)");
				printTaskSequence(*i);
				if(!runTaskSet(*i))
				{
					ROS_ERROR_STREAM(ros::this_node::getName()<<": Multithreaded Task(s) Returned Error");
					termination_error_found =  terminationErrorReceived(*i);
					completed = false;
					break;
				}
				ROS_INFO_STREAM(ros::this_node::getName()<<": Completed Multithreaded Task(s)");
			}

			return completed;
		}

		void printTaskSequence(ConcurrentTaskSet &set)
		{
			std::stringstream ss;
			ss<<"\n\tMultithreaded Tasks:\n";
			ConcurrentTaskSet::iterator j;
			int counter =1;
			for(j = set.begin(); j != set.end(); j++)
			{
				ss<<"\t\t- "<<"arm "<<counter<<": "<<j->name_<<"\n";
				counter++;
			}

			ROS_INFO_STREAM(ros::this_node::getName()<<ss.str());
		}

		/*
		 * cycle through task set array until an error is received.  Returns false only if a termination error
		 * is sent back, returns true otherwise.
		 */
		bool cycleTaskSequence(TaskSequence &task_seq)
		{
			bool proceed_to_next_cycle = true;
			bool termination_error = false;
			while(proceed_to_next_cycle)
			{
				TaskSequence::iterator i;
				for(i = task_seq.begin(); i != task_seq.end(); i++)
				{
					ROS_INFO_STREAM(ros::this_node::getName()<<": Requesting Multithreaded Task(s)");
					printTaskSequence(*i);
					if(!runTaskSet(*i))
					{
						proceed_to_next_cycle = false;
						termination_error =  terminationErrorReceived(*i);
						ROS_ERROR_STREAM(ros::this_node::getName()<<": Multithreaded Task(s) Returned Error");
						break;
					}
					ROS_INFO_STREAM(ros::this_node::getName()<<": Completed Multithreaded Task(s)");
				}
			}

			return !termination_error;
		}

	protected:

		bool terminationErrorReceived(ConcurrentTaskSet &task_set)
		{
			ConcurrentTaskSet::iterator i;
			bool termination = false;
			for(i = task_set.begin(); !termination && i != task_set.end(); i++)
			{
				termination = termination || i->terminationErrorReceived();
			}

			return termination;
		}
	};

public:
	ConcurrentArmMoveSupervisor():
		interrupt_cycle_(false),
		continue_on_cycle_fail_(false),
		max_cycle_count_(std::numeric_limits<int>::infinity())
	{
		initializeTaskDefinitions();
	}

	~ConcurrentArmMoveSupervisor()
	{

	}

	void run()
	{

		using namespace boost::assign;

		if(!setup())
		{
			return;
		}

		// designating clients for clutter and sort
		ros::ServiceClient &clutter_arm_client = arm1_handshaking_client_;
		ros::ServiceClient &sorting_arm_client = arm2_handshaking_client_;

		// creating concurrent task sequences
		TaskSequence clutter_start_seq, clutter_cycle_seq, clutter_end_seq;
		TaskSequence sort_start_seq, sort_cycle_seq, sort_end_seq;
		TaskSequence arm1_clear_singulated_zone_seq;
		TaskSequence arm2_clear_singulated_zone_seq;
		TaskSequence move_home_seq;

		generateMoveHomeSequence(clutter_arm_client,sorting_arm_client,move_home_seq);
		generateSortSequences(clutter_arm_client,sorting_arm_client,sort_cycle_seq,sort_start_seq,sort_end_seq);
		generateClutterSequences(clutter_arm_client,sorting_arm_client,clutter_cycle_seq,clutter_start_seq,clutter_end_seq);
		//generateSortSequences(sorting_arm_client,clutter_arm_client,clutter_cycle_seq,clutter_start_seq,clutter_end_seq);
		generateClearSingulationZoneSequence(clutter_arm_client,arm1_clear_singulated_zone_seq);
		generateClearSingulationZoneSequence(sorting_arm_client,arm2_clear_singulated_zone_seq);

		// creating sequence executor
		bool stop_running = false;
		TaskSetExecutor task_executor;

		// clearing singulation zone
		ROS_INFO_STREAM(": ------------------ Request Clutter Arm Clear Singulation ------------------ ");
		if(!task_executor.runTaskSequence(arm1_clear_singulated_zone_seq,stop_running) && stop_running)
		{
			ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
			return;
		}

		ROS_INFO_STREAM(": ------------------ Request Sorting Arm Clear Singulation ------------------ ");
		if(!task_executor.runTaskSequence(arm2_clear_singulated_zone_seq,stop_running) && stop_running)
		{
			ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
			return;
		}

		while(ros::ok() && checkServiceClientConnections())
		{
			// start by moving arms home
			ROS_INFO_STREAM(": ------------------ Request Move home ------------------ ");
			if(!task_executor.runTaskSequence(move_home_seq,stop_running))
			{
				ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
				break;
			}

			ROS_INFO_STREAM(": ------------------ Request Sort start sequence ------------------ ");
			if(task_executor.runTaskSequence(sort_start_seq,stop_running))
			{
				// cycle until sorting is finished
				ROS_INFO_STREAM(": ------------------ Request Sort Cycle sequence ------------------ ");
				if(!task_executor.cycleTaskSequence(sort_cycle_seq))
				{
					ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
					break;
				}

				ROS_INFO_STREAM(": ------------------ Request Sort End sequence ------------------ ");
				if(!task_executor.runTaskSequence(sort_end_seq,stop_running))
				{
					ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
					break;
				}
			}
			else
			{
				if(stop_running)
				{
					ROS_ERROR_STREAM(node_name_<<": Termination error received, exiting");
					break;
				}
			}

			// moving arms home (just in case)
			ROS_INFO_STREAM(": ------------------ Request Move Home ------------------ ");
			if(!task_executor.runTaskSequence(move_home_seq,stop_running))
			{
				ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
				break;
			}

			ROS_INFO_STREAM(": ------------------ Request Clutter start sequence ------------------ ");
			if(task_executor.runTaskSequence(clutter_start_seq,stop_running))
			{
				// cycle until clutter is finished
				ROS_INFO_STREAM(": ------------------  Request Clutter Cycle Sequence ------------------ ");
				if(!task_executor.cycleTaskSequence(clutter_cycle_seq))
				{
					ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
					break;
				}

				ROS_INFO_STREAM(": ------------------  Request Clutter End Sequence ------------------ ");
				if(!task_executor.runTaskSequence(clutter_end_seq,stop_running))
				{
					ROS_ERROR_STREAM(node_name_<<": Request not completed, exiting");
					break;
				}
			}
			else
			{
				if(stop_running)
				{
					ROS_ERROR_STREAM(node_name_<<": Termination error received, exiting");
					break;
				}
			}
		}
	}


	std::map<uint32_t,TaskDetails>& getTasksDefinitions()
	{
		return task_definitions_;
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

	std::map<uint32_t,TaskDetails> task_definitions_;// contains the building blocks for the sequences

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

		initializeTaskDefinitions();

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

		return true;
	}

	void initializeTaskDefinitions()
	{
		using namespace boost::assign;

		if(!task_definitions_.empty())
		{
			return;
		}

		uint32_t attempts = MAX_SERVICE_CALL_ATTEMPTS;
		TaskDetails t;
		std::vector<uint32_t> task_codes;
		ros::ServiceClient empty_client;

		// perception for singulation of clutter
		task_codes = list_of((uint32_t)ArmRequest::TASK_PERCEPTION_FOR_SINGULATION);
		t = TaskDetails("Singulate Perception",
				empty_client,
				task_codes,
				attempts);
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_PERCEPTION_FOR_SINGULATION,t));

		// perception for cluttering
		task_codes = list_of((uint32_t)ArmRequest::TASK_PERCEPTION_FOR_CLUTTERING);
		t = TaskDetails("Clutter Perception",
				empty_client,
				task_codes,
				attempts);
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_PERCEPTION_FOR_CLUTTERING,t));

		// perception for sorting
		task_codes = list_of((uint32_t)ArmRequest::TASK_PERCEPTION_FOR_SORTING);
		t = TaskDetails("Sorting Perception",
				empty_client,
				task_codes,
				attempts);
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_PERCEPTION_FOR_SORTING,t));

		// grasp planning for singulation
		task_codes = list_of((uint32_t)ArmRequest::TASK_GRASP_PLANNING_FOR_SINGULATION);
		t = TaskDetails("Singulation Grasp Planning",
				empty_client,
				task_codes,
				attempts);
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_GRASP_PLANNING_FOR_SINGULATION,t));

		// grasp planning for clutter
		task_codes = list_of((uint32_t)ArmRequest::TASK_GRASP_PLANNING_FOR_CLUTTER);
		t = TaskDetails("Sort Grasp Planning",
				empty_client,
				task_codes,
				attempts);
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_GRASP_PLANNING_FOR_CLUTTER,t));

		// grasp planning for sorting
		task_codes = list_of((uint32_t)ArmRequest::TASK_GRASP_PLANNING_FOR_SORT);
		t = TaskDetails("Sort Grasp Planning",
				empty_client,
				task_codes,
				attempts);
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_GRASP_PLANNING_FOR_SORT,t));

		// move home
		task_codes = list_of((uint32_t)ArmRequest::TASK_MOVE_HOME);
		t = TaskDetails("Move Home",
				empty_client,
				task_codes,
				1,// 1 attempt allowed
				ArmResponse::FINAL_MOVE_HOME_ERROR);// termination error code
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_MOVE_HOME,t));

		// move to pick
		TaskDetails move_recovery_task = t;
		task_codes = list_of((uint32_t)ArmRequest::TASK_MOVE_TO_PICK);
		t = TaskDetails("Move To Pick",
				empty_client,
				task_codes,
				1,// 1 attempt allowed
				ArmResponse::MOVE_COMPLETION_ERROR);// termination error code
		t.addRecoveryTask(false,move_recovery_task);// move home recovery task added
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_MOVE_TO_PICK,t));

		// move to place
		task_codes = list_of((uint32_t)ArmRequest::TASK_MOVE_TO_PLACE);
		t = TaskDetails("Move To Place",
				empty_client,
				task_codes,
				1,// 1 attempt allowed
				ArmResponse::MOVE_COMPLETION_ERROR);// termination error code
		t.addRecoveryTask(false,move_recovery_task);// move home recovery task added
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_MOVE_TO_PLACE,t));

		// move to place and then home
		task_codes = list_of((uint32_t)ArmRequest::TASK_MOVE_TO_PLACE)
				((uint32_t)ArmRequest::TASK_MOVE_HOME);
		t = TaskDetails("Move To Place Then Home",
				empty_client,
				task_codes,
				1,// 1 attempt allowed
				ArmResponse::MOVE_COMPLETION_ERROR);// termination error code
		t.addRecoveryTask(false,move_recovery_task);// move home recovery task added
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_MOVE_TO_PLACE_THEN_HOME,t));

		// move to pick place and then home
		task_codes = list_of((uint32_t)ArmRequest::TASK_MOVE_TO_PICK)
				((uint32_t)ArmRequest::TASK_MOVE_TO_PLACE)
				((uint32_t)ArmRequest::TASK_MOVE_HOME);
		t = TaskDetails("Move To Pick Place Then Home",
				empty_client,
				task_codes,
				1,// 1 attempt allowed
				ArmResponse::MOVE_COMPLETION_ERROR);// termination error code
		t.addRecoveryTask(false,move_recovery_task);// move home recovery task added
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_MOVE_TO_PICK_PLACE_THEN_HOME,t));

		// clear results
		task_codes = list_of((uint32_t)ArmRequest::TASK_CLEAR_RESULTS);
		t = TaskDetails("Clear Results",
				empty_client,
				task_codes,
				1);// 1 attempt allowed
		task_definitions_.insert(std::make_pair(ArmRequest::TASK_CLEAR_RESULTS,t));

	}

	void generateSortSequences(ros::ServiceClient &clutter_client,ros::ServiceClient &sort_client,
			TaskSequence &sort_cycle_seq,TaskSequence &sort_start_seq,TaskSequence &sort_end_seq)
	{
		// defining task sets
		ConcurrentTaskSet set;
		TaskDetails task1, task2;

		/* --------------------------------- Sort Cyclical Sequence definition -------------------------
		 * clutter arm client starts at home position and there's one object in the singulation area
		*/
		// clear data
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task2 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task1.arm_client_ = clutter_client;
//		task2.arm_client_ = sort_client;
//		set.push_back(task1);
//		set.push_back(task2);
//		sort_cycle_seq.push_back(set);

		// perception in clutter and singulated zones
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SINGULATION];
//		task2 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SORTING];
//		task1.arm_client_ = clutter_client;
//		task2.arm_client_ = sort_client;
//		set.push_back(task1);
//		set.push_back(task2);
//		sort_cycle_seq.push_back(set);

		// grasp planning in clutter and singulation
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SINGULATION];
		task2 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SORT];
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		sort_cycle_seq.push_back(set);

		// move to pick in clutter and singulation zone
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK];// should be at home and moving to clutter pick zone
		task2 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK];// should be at home and moving to singulation zone
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		sort_cycle_seq.push_back(set);

		// move to place (sort client moves part from singulation to sorted zone)
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PLACE];// moves from singulation to sorted zone
		task1.arm_client_ = sort_client;
		set.push_back(task1);
		sort_cycle_seq.push_back(set);

		// move to place (clutter to singulation) and move home
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PLACE_THEN_HOME];// should be at clutter pick and moving to singulation zone
		task2 = task_definitions_[ArmRequest::TASK_MOVE_HOME];// should be at sorted and moving to home
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		sort_cycle_seq.push_back(set);

		/*
		 * ---------------------------- End of Clutter Cyclical Sequence definition -------------------------
		*/

		/*
		 * --------------------------------- Sort Start Sequence definition -------------------------
		 * Moves first object from clutter to singulated and returns home
		 */

		// clear data
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task2 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task1.arm_client_ = sort_client;
//		task2.arm_client_ = clutter_client;
//		set.push_back(task1);
//		set.push_back(task2);
//		sort_start_seq.push_back(set);

		// perception in clutter zone
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SINGULATION];
//		task1.arm_client_ = clutter_client;
//		set.push_back(task1);
//		sort_start_seq.push_back(set);

		// perception and grasp planning from clutter to singulated zone
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SINGULATION];
		task1.arm_client_ = clutter_client;
		set.push_back(task1);
		sort_start_seq.push_back(set);

		// move to pick place then home
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK_PLACE_THEN_HOME];
		task1.arm_client_ = clutter_client;
		set.push_back(task1);
		sort_start_seq.push_back(set);

		/*
		 * --------------------------------- Sort End Sequence definition -------------------------
		 * Moves last object from singulated to sorted and returns home
		 */

		// clear data
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task1.arm_client_ = sort_client;
//		set.push_back(task1);
//		sort_end_seq.push_back(set);

		// perception in singulated zone
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SORTING];
//		task1.arm_client_ = sort_client;
//		set.push_back(task1);
//		sort_end_seq.push_back(set);

		// grasp planning from singulated to sorted zone
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SORT];
		task1.arm_client_ = sort_client;
		set.push_back(task1);
		sort_end_seq.push_back(set);

		// move to pick place then home
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK_PLACE_THEN_HOME];
		task1.arm_client_ = sort_client;
		set.push_back(task1);
		sort_end_seq.push_back(set);
	}

	void generateClutterSequences(ros::ServiceClient &clutter_client,ros::ServiceClient &sort_client,
			TaskSequence &clutter_cycle_seq,TaskSequence &clutter_start_seq,TaskSequence &clutter_end_seq)
	{

		// defining task sets
		ConcurrentTaskSet set;
		TaskDetails task1, task2;

		/* --------------------------------- Clutter Cyclical Sequence definition -------------------------
		 * sort arm client starts at home position and there's one object in the singulation area
		*/
		// clear data
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task2 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task1.arm_client_ = clutter_client;
//		task2.arm_client_ = sort_client;
//		set.push_back(task1);
//		set.push_back(task2);
//		clutter_cycle_seq.push_back(set);

		// perception in sorted and singulated zones
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_CLUTTERING];// segmentation in singulation zone
//		task2 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SINGULATION];// segmentation in sorted zone
//		task1.arm_client_ = clutter_client;
//		task2.arm_client_ = sort_client;
//		set.push_back(task1);
//		set.push_back(task2);
//		clutter_cycle_seq.push_back(set);

		// perception and grasp planning in sorted and singulation
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_CLUTTER];
		task2 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SINGULATION];
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		clutter_cycle_seq.push_back(set);

		// move to pick in clutter and singulation zone
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK];// should be at home and moving to singulated pick zone
		task2 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK];// should be at home and moving to sorted zone
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		clutter_cycle_seq.push_back(set);

		// move to place (sort client moves part from singulation to sorted zone)
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PLACE];// moves from singulation to clutter zone
		task1.arm_client_ = clutter_client;
		set.push_back(task1);
		clutter_cycle_seq.push_back(set);

		// move to place (clutter to singulation) and move home
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_HOME];// should be at clutter and moving home
		task2 = task_definitions_[ArmRequest::TASK_MOVE_TO_PLACE_THEN_HOME];// should be at sorted and moving singulation then home
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		clutter_cycle_seq.push_back(set);

		/*
		 * ---------------------------- End of Clutter Cyclical Sequence definition -------------------------
		*/

		/*
		 * --------------------------------- Clutter Start Sequence definition -------------------------
		 * Moves first object from sorted to singulated and returns home
		 */

		// clear data
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task2 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task1.arm_client_ = sort_client;
//		task2.arm_client_ = clutter_client;
//		set.push_back(task1);
//		set.push_back(task2);
//		clutter_start_seq.push_back(set);

		// perception in sorted zone
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SINGULATION];
//		task1.arm_client_ = sort_client;
//		set.push_back(task1);
//		clutter_start_seq.push_back(set);

		// perception and grasp planning from clutter to singulated zone
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SINGULATION];
		task1.arm_client_ = sort_client;
		set.push_back(task1);
		clutter_start_seq.push_back(set);

		// move to pick place then home
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK_PLACE_THEN_HOME];
		task1.arm_client_ = sort_client;
		set.push_back(task1);
		clutter_start_seq.push_back(set);

		/*
		 * --------------------------------- Clutter Start Sequence definition -------------------------
		 * Moves last object from singulated to clutter and returns home
		 */

		// clear data
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
//		task1.arm_client_ = clutter_client;
//		set.push_back(task1);
//		clutter_end_seq.push_back(set);

		// perception in singulated zone
//		set.clear();
//		task1 = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_CLUTTERING];
//		task1.arm_client_ = clutter_client;
//		set.push_back(task1);
//		clutter_end_seq.push_back(set);

		// perception and grasp planning from singulated to clutter zone
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_CLUTTER];
		task1.arm_client_ = clutter_client;
		set.push_back(task1);
		clutter_end_seq.push_back(set);

		// move to pick place then home
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK_PLACE_THEN_HOME];
		task1.arm_client_ = clutter_client;
		set.push_back(task1);
		clutter_end_seq.push_back(set);
	}

	void generateClearSingulationZoneSequence(ros::ServiceClient &client, TaskSequence &seq)
	{
		// defining task sets
		ConcurrentTaskSet set;
		TaskDetails task;

		// clear data
		set.clear();
		task = task_definitions_[ArmRequest::TASK_CLEAR_RESULTS];
		task.arm_client_ = client;
		set.push_back(task);
		seq.push_back(set);

		// move home
		set.clear();
		task = task_definitions_[ArmRequest::TASK_MOVE_HOME];
		task.arm_client_ = client;
		set.push_back(task);
		seq.push_back(set);

		// perception
//		set.clear();
//		task = task_definitions_[ArmRequest::TASK_PERCEPTION_FOR_SORTING];
//		task.arm_client_ = client;
//		set.push_back(task);
//		seq.push_back(set);

		// perception and grasp planning
		set.clear();
		task = task_definitions_[ArmRequest::TASK_GRASP_PLANNING_FOR_SORT];
		task.arm_client_ = client;
		set.push_back(task);
		seq.push_back(set);

		// move pick place then home
		set.clear();
		task = task_definitions_[ArmRequest::TASK_MOVE_TO_PICK_PLACE_THEN_HOME];
		task.arm_client_ = client;
		set.push_back(task);
		seq.push_back(set);

	}

	void generateMoveHomeSequence(ros::ServiceClient &clutter_client,ros::ServiceClient &sort_client,
			TaskSequence &seq)
	{
		// defining task sets
		ConcurrentTaskSet set;
		TaskDetails task1, task2;

		// clear data
		set.clear();
		task1 = task_definitions_[ArmRequest::TASK_MOVE_HOME];
		task2 = task_definitions_[ArmRequest::TASK_MOVE_HOME];
		task1.arm_client_ = clutter_client;
		task2.arm_client_ = sort_client;
		set.push_back(task1);
		set.push_back(task2);
		seq.push_back(set);
	}
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"concurrent_arm_move_supervisor_node");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ConcurrentArmMoveSupervisor arm_supervisor;
	arm_supervisor.run();

	spinner.stop();

	return 0;
}
