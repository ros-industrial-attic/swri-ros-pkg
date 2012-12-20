/*
 * TestOverheadGraspPlanner.cpp
 *
 *  Created on: Aug 15, 2012
 *      Author: jnicho
 */
#include <ros/ros.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


const std::string NO_PARENT_FRAME_ID = "/NO_PARENT";
const std::string SERVICE_NAME = "plan_point_cluster_grasp";
const std::string WORLD_FRAME_ID = "base_link";
const std::string CLUSTER_FRAME_ID = "cluster";
const std::string POINT_CLOUD_TOPIC_NAME = "cloud_in";


class TestPlanner
{
public:
	TestPlanner():
	_serviceName(SERVICE_NAME),
	_worldFrameId(WORLD_FRAME_ID),
	_clusterFrameId(CLUSTER_FRAME_ID),
	_clusterTopicName(POINT_CLOUD_TOPIC_NAME)
	{

	}

	~TestPlanner()
	{

	}

	void fetchParams()
	{
		std::string nodeNamespace = "/" + ros::this_node::getName();
		ros::param::param(nodeNamespace + "/world_frame_id",_worldFrameId,WORLD_FRAME_ID);
		//ros::param::param(nodeNamespace + "/cluster_frame_id",_clusterFrameId,CLUSTER_FRAME_ID);
	}

	void init()
	{
		ros::NodeHandle nh;

		_serviceClient = nh.serviceClient<object_manipulation_msgs::GraspPlanning>(_serviceName);
		_clusterSubscriber = nh.subscribe(_clusterTopicName,1,&TestPlanner::clusterCallback,this);

		tf::Transform worldTransform = tf::Transform::getIdentity();

		ros::Duration loopCycle(0.2f);
		std::vector<tf::StampedTransform> transforms;
		while(ros::ok())
		{
			fetchParams();
			//transforms.push_back(tf::StampedTransform(worldTransform,ros::Time::now(),"my_world_frame",_worldFrameId));
			//transforms.push_back(tf::StampedTransform(worldTransform,ros::Time::now(),_worldFrameId,_clusterFrameId));
			//_tfBroadcaster.sendTransform(tf::StampedTransform(worldTransform,ros::Time::now(),"world",_worldFrameId));
			//_tfBroadcaster.sendTransform(tf::StampedTransform(worldTransform,ros::Time::now(),_worldFrameId,_clusterFrameId));
			//_tfBroadcaster.sendTransform(transforms);
			ros::spinOnce();
			loopCycle.sleep();
			transforms.clear();
		}
	}

	void clusterCallback(const sensor_msgs::PointCloud2ConstPtr cloudMsg)
	{
		// creating grasp planning service request;
		object_manipulation_msgs::GraspPlanning srv;
		sensor_msgs::PointCloud inCloud;
		sensor_msgs::convertPointCloud2ToPointCloud(*cloudMsg,inCloud);
		//inCloud.header.frame_id = _clusterFrameId;
		_clusterFrameId = inCloud.header.frame_id;
		srv.request.target.cluster = inCloud;
		srv.request.target.reference_frame_id = _worldFrameId;

		if(_serviceClient.call(srv))
		{
			ROS_INFO("%s",std::string(_serviceName + " call returned grasp plan").c_str());
		}
		else
		{
			ROS_ERROR("%s",std::string(_serviceName + " call fail to resolve grasp plan").c_str());
		}
	}

protected:
	// service
	ros::ServiceClient _serviceClient;
	std::string _serviceName;

	// subscriber
	ros::Subscriber _clusterSubscriber;
	std::string _clusterTopicName;

	// transforms
	tf::TransformBroadcaster _tfBroadcaster;
	std::string _worldFrameId;
	std::string _clusterFrameId;

};
int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_overhead_grasp_planner");
	ros::NodeHandle nh;

	TestPlanner testPlaner = TestPlanner();
	testPlaner.init();

	return 0;
}

