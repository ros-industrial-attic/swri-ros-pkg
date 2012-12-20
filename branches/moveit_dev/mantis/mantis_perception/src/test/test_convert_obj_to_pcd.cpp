/*
 * test_convert_obj_to_pcd.cpp
 *
 *  Created on: Oct 24, 2012
 */

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointNormal> NormalsCloud;

void printCloud(const NormalsCloud &cloud)
{
	std::cout<<"\nCloud Data"<<"\n";
	std::cout<<"\tSize: "<<cloud.size()<<"\n";
	std::cout<<"\tWidth"<<cloud.width<<"\n";
	std::cout<<"\tHeight"<<cloud.height<<"\n";
	std::cout<<"\tPoints List\n";

	for(int i = 0 ; i < 200; i++)
	{
		const pcl::PointNormal &p = cloud[i];
		std::cout<<"\t\tp: "<<p.x<<", "<<p.y<<", "<<p.z<<"; normal: "<<p.normal_x<<", "<<p.normal_y<<", "<<p.normal_z<<"\n";
	}

}

void printCloud(const Cloud &cloud)
{
	std::cout<<"\nCloud Data"<<"\n";
	std::cout<<"\tSize: "<<cloud.size()<<"\n";
	std::cout<<"\tWidth"<<cloud.width<<"\n";
	std::cout<<"\tHeight"<<cloud.height<<"\n";
	std::cout<<"\tPoints List\n";

	for(int i = 0 ; i < 200; i++)
	{
		const pcl::PointXYZ &p = cloud[i];
		std::cout<<"\t\tp: "<<p.x<<", "<<p.y<<", "<<p.z<<"\n";
	}
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_convert_obj_to_pcd");
	ros::NodeHandle nh;
	std::string nodeName = ros::this_node::getName();

	// parsing arguments
	if(argc == 1)
	{
		ROS_ERROR_STREAM(nodeName<<": did not pass path to file as argument, exiting");
		return 0;
	}

	// reading file and storing it in cloud
	NormalsCloud normCloud;
	Cloud cloud;
	std::string fileName = std::string(argv[1]);
	bool success = false;

	// reading with normals first
	if(pcl::io::loadPCDFile<pcl::PointNormal>(fileName,normCloud) == -1)
	{
		ROS_ERROR_STREAM(nodeName<<": could read file "<<fileName<<" as normals and points");
	}
	else
	{
		ROS_INFO_STREAM(nodeName<<": successfully read file "<<fileName<< " as set of normal vectors and points");
		printCloud(normCloud);
		success = true;
	}


	if(!success && pcl::io::loadPCDFile<pcl::PointXYZ>(fileName,cloud) == -1)
	{
		ROS_ERROR_STREAM(nodeName<<": could read file "<<fileName<<" as  points");
	}
	else
	{
		ROS_INFO_STREAM(nodeName<<": successfully read file "<<fileName<< " as set of points");
		printCloud(cloud);
	}
}



