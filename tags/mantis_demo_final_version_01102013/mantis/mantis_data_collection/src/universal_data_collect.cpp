#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
//#include "mantis_data_collection/dataCollect.h"
#include "mantis_data_collection/process_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <vector>

//ros::Publisher pan_pub;
//ros::Publisher vis_pub;
ros::ServiceClient cloud_process_client;
sensor_msgs::PointCloud2 cloud_to_process;
sensor_msgs::JointState last_joint_state;
bool joint_state_initialized = false;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;


void kinect_cb(sensor_msgs::PointCloud2 fromKinect)
{
	cloud_to_process = fromKinect;
}

void joint_state_cb(sensor_msgs::JointState joints)
{
    if (!joint_state_initialized)
    {
	    last_joint_state = joints;
	    joint_state_initialized = true;
	    ROS_INFO("Joint state initialized");
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "universal_data_collect");
	ros::NodeHandle n;

	//pan_pub = n.advertise<std_msgs::UInt16>("/pan_command",1);
	//ros::ServiceServer pan_serv = n.advertiseService("/pan360_data_collect", rotate_cb);
	ros::Subscriber pan_sub = n.subscribe("/ur5_arm_kinect/depth_registered/points", 1, kinect_cb);
	ros::Subscriber js_sub = n.subscribe("ur5_arm/joint_states", 1, joint_state_cb);
	cloud_process_client = n.serviceClient<mantis_data_collection::process_cloud>("process_cloud");
	//vis_pub = n.advertise<visualization_msgs::Marker>( "matching_mesh_marker", 10 );

	ros::NodeHandle private_nh("~");
	std::string object_name_;
	const std::string PARAM_OBJ_NAME_DEFAULT= "object_name";
	const std::string OBJ_NAME_DEFAULT= "fake_object";
	private_nh.param(PARAM_OBJ_NAME_DEFAULT, object_name_, OBJ_NAME_DEFAULT);

	ROS_INFO("Ready!");

	// Waiting for joint state to initialize.  The joint state at init will be used for all
	// move requests.  This will allow the user to set the arm configuration before the node
	// is executed.
        ros::Rate loop_rate(1);
	while(!joint_state_initialized && ros::ok())
	{
          ros::spinOnce();  // Need to spin in order to process joint state messages
	  ROS_INFO("Waiting for joint state to initialize");
	  loop_rate.sleep();
	}

	ROS_INFO("Creating client and waiting for server");
	Client client("/ur5_arm/joint_trajectory_action");
        client.waitForServer();

        // Create and populate goal with default values
        ROS_INFO("Populating goal with default values");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = last_joint_state.name;
        int last_joint_index = last_joint_state.name.size() - 1;


        // Increment from -180 to 180 at 3 degree increments on last joint
        for(double joint_angle = -3.14; joint_angle <= 3.14; joint_angle = joint_angle + ((3.0*3.14)/180.0))
        {

        	ROS_INFO_STREAM("Currently at position " << joint_angle*180/3.14);
        	mantis_data_collection::process_cloud srv;

        	srv.request.objectName = object_name_;
        	srv.request.in_cloud = cloud_to_process;
        	double request_angle=joint_angle*180.0/3.14+180.0;
        	srv.request.angle = std::floor(request_angle);
        	ROS_INFO_STREAM("Sending object "<<object_name_<< " with angle "<<std::floor(request_angle)<<" to feature extractor");
        	//pan_client.call(srv);
        	if (!cloud_process_client.call(srv))
        	    {
        	      ROS_ERROR("Call to feature extractor/cloud processor service failed");
        	    }

        	srv.response.result = 1;

          loop_rate.sleep(); //Delaying between moves.
          ROS_INFO("Populating next point");
          trajectory_msgs::JointTrajectoryPoint pt;
          pt.positions = last_joint_state.position;
          pt.positions[last_joint_index] = joint_angle;
          // Velocities and accelerations must be set to zero.  If they aren't than
          // the arm tries to move at the desired velocity regardless of the position.
          pt.velocities.clear();
          pt.velocities.resize(pt.positions.size(), 0.0);
          //pt.velocities[last_joint_index] = 0.25;
          pt.accelerations.clear();
          pt.accelerations.resize(pt.positions.size(), 0.0);
          //pt.accelerations[last_joint_index] = 2.5;
          pt.time_from_start = ros::Duration(1.0);


          // Cache the last joint state;
          ROS_INFO("Updating last joint state");
          last_joint_state.position = pt.positions;

          // Clear out the last trajectory we sent

          ROS_INFO("Clearing out last trajectory");
          goal.trajectory.points.clear();
          // Add single point to trajectory.

          ROS_INFO("Creating new trajectory with single point");
          goal.trajectory.points.push_back(pt);
          goal.trajectory.header.stamp = ros::Time::now();

          ROS_INFO_STREAM("Sending goal angle: " << joint_angle);

          client.sendGoal(goal);
          while(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
          {
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO("Waiting for client to finish");
          }

          ROS_INFO_STREAM("Joint angle:" << joint_angle << " achieved");

          if (!ros::ok()) break;
        }


	return 0;
}


