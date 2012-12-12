#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include "mantis_data_collection/dataCollect.h"
#include "mantis_data_collection/process_cloud.h"
#include <sensor_msgs/PointCloud2.h>_
#include <sstream>
#include <visualization_msgs/Marker.h>

ros::Publisher pan_pub;
ros::Publisher vis_pub;
ros::ServiceClient pan_client;
sensor_msgs::PointCloud2 cloud_to_process;

void kinect_cb(sensor_msgs::PointCloud2 fromKinect)
{
	cloud_to_process = fromKinect;
}

bool rotate_cb(mantis_data_collection::dataCollect::Request &req, mantis_data_collection::dataCollect::Response &res)
{
std_msgs::UInt16 command;
command.data=0;
mantis_data_collection::process_cloud srv;

srv.request.objectName = req.objectName;
//following line added by CG along with following // comments to make useful for one angle
command.data = req.delta;

//while(command.data<360)
//{
//ros::Rate loop_rate(.1);

srv.request.in_cloud = cloud_to_process;
srv.request.angle = command.data;
pan_client.call(srv);
srv.response.result = 1;

pan_pub.publish(command);
//std::cout << "Angle " << srv.request.angle << "\n";
//ros::spinOnce();
//loop_rate.sleep();
//command.data += req.delta;
//}

res.result = 1;
std::cout << "Status: " << srv.response.result << "\n";
return 1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pan360_data_collect");
	ros::NodeHandle n;

	pan_pub = n.advertise<std_msgs::UInt16>("/pan_command",1);
	ros::ServiceServer pan_serv = n.advertiseService("/pan360_data_collect", rotate_cb);
	ros::Subscriber pan_sub = n.subscribe("/camera/depth_registered/points", 1, kinect_cb);
	pan_client = n.serviceClient<mantis_data_collection::process_cloud>("process_cloud");
	vis_pub = n.advertise<visualization_msgs::Marker>( "matching_mesh_marker", 10 );

	ROS_INFO("Ready!");
	ros::spin();


	return 0;
}


