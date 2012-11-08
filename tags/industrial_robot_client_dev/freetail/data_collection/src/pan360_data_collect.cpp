#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include "data_collection/dataCollect.h"
#include "data_collection/process_cloud.h"
#include <sensor_msgs/PointCloud2.h>_
#include <sstream>

ros::Publisher pan_pub;
ros::ServiceClient pan_client;
sensor_msgs::PointCloud2 cloud_to_process;

void kinect_cb(sensor_msgs::PointCloud2 fromKinect)
{
cloud_to_process = fromKinect;
}

bool rotate_cb(data_collection::dataCollect::Request &req, data_collection::dataCollect::Response &res)
{
std_msgs::UInt16 command;
command.data=0;
data_collection::process_cloud srv;

srv.request.objectName = req.objectName;

while(command.data<360)
{
ros::Rate loop_rate(.1);

srv.request.in_cloud = cloud_to_process;
srv.request.angle = command.data;
pan_client.call(srv);
srv.response.result = 1;

pan_pub.publish(command);
//std::cout << "Angle " << srv.request.angle << "\n";
ros::spinOnce();
loop_rate.sleep();
command.data += req.delta;
}

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
pan_client = n.serviceClient<data_collection::process_cloud>("process_cloud");


ROS_INFO("Ready!");
ros::spin();


return 0;
}


