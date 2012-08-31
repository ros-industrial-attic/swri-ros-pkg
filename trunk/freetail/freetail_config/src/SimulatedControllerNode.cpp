/*
 * SimulatedControllerNode.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: jnicho
 *
 *  Description:
 *  	This node should allow faking a robot controller that communicates with the "joint_trajectory_action" server.
 *  	It subscribes to the topic "command" of type "trajectory_msgs/JointTrajectory"
 *  	It publishes the topic "state" of type "pr2_controllers_msgs/JointTrajectoryControllerState"
 *  	The command and state topics are published and advertised by the joint_trajectory_action server
 */

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <boost/foreach.hpp>

const std::string CONTROLLER_STATE_TOPIC_NAME = "state";
const std::string CONTROLLER_FEEDBACK_TOPIC_NAME = "feedback_states";
const std::string JOINT_TRAJECTORY_TOPIC_NAME = "command";
const std::string JOINT_STATE_TOPIC_NAME = "joint_states";
const std::string JOINT_NAMES_PARAM_NAME = "/joint_trajectory_action/joints";

class SimulatedController
{
public:
	SimulatedController()
	:_CntrlStatePubTopic(CONTROLLER_STATE_TOPIC_NAME),
	 _ControlFeedbackPubTopic(CONTROLLER_FEEDBACK_TOPIC_NAME),
	_JointStatePubTopic(JOINT_STATE_TOPIC_NAME),
	_JointTrajSubsTopic(JOINT_TRAJECTORY_TOPIC_NAME),
	_JointNames(),
	_LastJointState(),
	_ProcessingRequest(false)
	{

	}

	~SimulatedController()
	{

	}

	void init()
	{
		ros::NodeHandle nh;
		std::string nodeName = ros::this_node::getName();

		// finding joint names
		std::string matchFound;
		XmlRpc::XmlRpcValue list;
		_JointNames.clear();
		_LastJointState.name.clear();
		_LastJointState.position.clear();
		_LastJointState.velocity.clear();
		_LastJointState.effort.clear();

		if(ros::param::search(JOINT_NAMES_PARAM_NAME,matchFound))
		{
			if(nh.getParam(matchFound,list))
			{
				if(list.getType() == XmlRpc::XmlRpcValue::TypeArray)
				{
					ROS_INFO("%s",std::string(nodeName + ": Found joints under parameter " + matchFound).c_str());
					//BOOST_FOREACH(XmlRpc::XmlRpcValue val,list.)
					for(int i = 0; i < list.size(); i++)
					{
						XmlRpc::XmlRpcValue val = list[i];
						if(val.getType() == XmlRpc::XmlRpcValue::TypeString)
						{
							std::string name = static_cast<std::string>(val);
							_JointNames.push_back(name);
							_LastJointState.name.push_back(name);
							_LastJointState.position.push_back(0.0f);
							_LastJointState.velocity.push_back(0.0f);
							_LastJointState.effort.push_back(0.0f);
						}
					}
				}
			}
			else
			{
				ROS_INFO("%s",std::string(nodeName + ": Could not retrieve parameter " + matchFound).c_str());
			}
		}
		else
		{
			ROS_INFO("%s",std::string(nodeName + ": Could not find parameter " + JOINT_NAMES_PARAM_NAME).c_str());
		}

		sensor_msgs::JointState st = _LastJointState;
		updateLastStateFeedbackMessages(st);

		_JointTrajSubscriber = nh.subscribe(_JointTrajSubsTopic,1,&SimulatedController::callbackJointTrajectory,this);
		_ControllerStatePublisher = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(_CntrlStatePubTopic,
				1);
		_ControllerFeedbackPublisher = nh.advertise<control_msgs::FollowJointTrajectoryFeedback>(_ControlFeedbackPubTopic,1);
		_JointStatePublisher = nh.advertise<sensor_msgs::JointState>(_JointStatePubTopic,1);
		_StatusUpdateTimer = nh.createTimer(ros::Duration(1.0f),&SimulatedController::broadcastState,this);
		ros::spin();
	}

	void broadcastState(const ros::TimerEvent &evnt)
	{
		//pr2_controllers_msgs::JointTrajectoryControllerState state;
		if(_ProcessingRequest)
		{
			return;
		}
		else
		{
			_LastControllerJointState.header.stamp = _LastControllerTrajState.header.stamp =  ros::Time::now();
			_LastJointState.header.stamp = ros::Time::now();
			_ControllerStatePublisher.publish(_LastControllerJointState);
			_ControllerFeedbackPublisher.publish(_LastControllerTrajState);
			_JointStatePublisher.publish(_LastJointState);
		}
	}

	void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
	{
		std::string nodeName = ros::this_node::getName();


		if(msg->points.empty())
		{
			_ProcessingRequest = false;
			return;
		}
		else
		{
			_ProcessingRequest = true;
		}

		ROS_INFO_STREAM(nodeName<<": Starting joint trajectory execution, "
				<<msg->points.size()<<" points in requested trajectory");

		// the messages below contain the same information, but the are expected by different joint trajectory action servers
		pr2_controllers_msgs::JointTrajectoryControllerState state;
		control_msgs::FollowJointTrajectoryFeedback feedback;

		trajectory_msgs::JointTrajectoryPoint error;
		state.joint_names = feedback.joint_names = _JointNames;
		error.positions = std::vector<double>(state.joint_names.size(),0.0f);
		state.error = feedback.error = error;

		sensor_msgs::JointState jointState;
		jointState.name = state.joint_names;

		//jointState.effort = std::vector<double>(state.joint_names.size(),0.0f);

		// publishing controller and joint states
		ros::Duration duration(0.0f);
		const ros::Duration printInterval(2.0f);
		ros::Time printStartTime = ros::Time::now();
		int counter = 0;
		BOOST_FOREACH(trajectory_msgs::JointTrajectoryPoint point,msg->points)
		{
			//trajectory_msgs::JointTrajectoryPoint point = msg->points[i];
			state.desired = feedback.desired = point;
			state.actual = feedback.actual = point;
			jointState.position = state.desired.positions;
			jointState.velocity = state.desired.velocities;

			(point.time_from_start - duration).sleep();
			duration = point.time_from_start;

			state.header.stamp = feedback.header.stamp = ros::Time::now();
			jointState.header.stamp = ros::Time::now();

			_ControllerStatePublisher.publish(state);
			_ControllerFeedbackPublisher.publish(feedback);
			_JointStatePublisher.publish(jointState);

			if(ros::Time::now() - printStartTime > printInterval)
			{
				ROS_INFO_STREAM(nodeName<<": Executed "<<
						counter<<" joint trajectory points after "<<point.time_from_start.sec<<" seconds");
				printStartTime = ros::Time::now();
			}
			counter++;
		}

		ROS_INFO_STREAM(nodeName<<": Finished joint trajectory execution, ");

		updateLastStateFeedbackMessages(jointState);
		//_LastJointState = jointState;
		_ProcessingRequest = false;
	}

protected:

	void updateLastStateFeedbackMessages(const sensor_msgs::JointState &st)
	{
		_LastJointState = st;

		trajectory_msgs::JointTrajectoryPoint error;
		error.positions = std::vector<double>(st.name.size(),0.0f);
		error.velocities = std::vector<double>(st.name.size(),0.0f);

		_LastControllerTrajState.joint_names = st.name;
		_LastControllerTrajState.actual.positions = _LastControllerTrajState.desired.positions = st.position;
		_LastControllerTrajState.actual.velocities = _LastControllerTrajState.desired.velocities = st.velocity;
		_LastControllerTrajState.actual.time_from_start = _LastControllerTrajState.desired.time_from_start = ros::Duration(0.0f);
		_LastControllerTrajState.error = error;

		_LastControllerJointState.joint_names = st.name;
		_LastControllerJointState.actual.positions = _LastControllerJointState.desired.positions = st.position;
		_LastControllerJointState.actual.velocities = _LastControllerJointState.desired.velocities = st.velocity;
		_LastControllerJointState.actual.time_from_start = _LastControllerJointState.desired.time_from_start = ros::Duration(0.0f);
		_LastControllerJointState.error = error;
	}

	// ros comm
	ros::Subscriber _JointTrajSubscriber;
	ros::Publisher _ControllerStatePublisher;
	ros::Publisher _JointStatePublisher;
	ros::Publisher _ControllerFeedbackPublisher;
	ros::Timer _StatusUpdateTimer;

	// topic names
	std::string _CntrlStatePubTopic;
	std::string _ControlFeedbackPubTopic;
	std::string _JointStatePubTopic;
	std::string _JointTrajSubsTopic;

	// ros parameters
	std::vector<std::string> _JointNames;

	// last states
	sensor_msgs::JointState _LastJointState;
	control_msgs::FollowJointTrajectoryFeedback _LastControllerTrajState;
	pr2_controllers_msgs::JointTrajectoryControllerState _LastControllerJointState;

	// others
	bool _ProcessingRequest;
};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"simulated_controller");
	ros::NodeHandle nh;
	SimulatedController simController = SimulatedController();
	simController.init();
	return 0;
}
