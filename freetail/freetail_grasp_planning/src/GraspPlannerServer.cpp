/*
 * GraspPlannerServer.cpp
 *
 *  Created on: Aug 9, 2012
 *      Author: jnicho
 */

#include "servers/GraspPlannerServer.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <boost/foreach.hpp>

GraspPlannerServer::GraspPlannerServer():
_GraspPlanner(0),
_ServiceName(SERVICE_NAME),
_PublishResults(false),
_PosePubTopic(POSE_PUBLISH_TOPIC_NAME),
_PlanePubTopic(PLANE_PUBLISH_TOPIC_NAME)
{
	// TODO Auto-generated constructor stub
}

GraspPlannerServer::~GraspPlannerServer() {
	// TODO Auto-generated destructor stub
	finish();
}

void GraspPlannerServer::init()
{
	ros::NodeHandle nh;
	// setting up grasp planner
	_GraspPlanner = new OverheadGraspPlanner();

	// fetching parameters from ros
	std::string nameSpace = "/" + ros::this_node::getName();
	ros::param::param(nameSpace + "/" + PARAM_NAME_PUBLISH_RESULTS,_PublishResults,false);

	// seting up ros server
	_ServiceServer = nh.advertiseService(_ServiceName,&GraspPlannerServer::serviceCallback,this);
	ROS_INFO("%s",std::string(_GraspPlanner->getPlannerName() + " advertising service: " + _ServiceName).c_str());

	// setting up ros publishers
	_PlanePublisher = nh.advertise<geometry_msgs::Polygon>(nameSpace + "/" + _PlanePubTopic,1);
	_PosePublisher = nh.advertise<geometry_msgs::PoseArray>(nameSpace + "/" + _PosePubTopic,1);
	ros::spin();
}

void GraspPlannerServer::finish()
{
	if(_GraspPlanner)
	{
		delete _GraspPlanner;
		_GraspPlanner = NULL;
	}
}

bool GraspPlannerServer::serviceCallback(object_manipulation_msgs::GraspPlanning::Request &req,
		object_manipulation_msgs::GraspPlanning::Response &res)
{
	if(_GraspPlanner)
	{
		bool success = _GraspPlanner->planGrasp(req,res);

		// retrieving parameter from ros
		std::string nameSpace = "/" + ros::this_node::getName();
		ros::param::param(nameSpace + "/" + PARAM_NAME_PUBLISH_RESULTS,_PublishResults,_PublishResults);
		if(success && _PublishResults)
		{
			// retrieving frameid
			std::string worldFrameId = req.target.reference_frame_id;

			// publishing poses
			geometry_msgs::PoseArray poses_msg;
			poses_msg.header.frame_id = worldFrameId;
			poses_msg.header.stamp = ros::Time::now();
			poses_msg.poses = std::vector<geometry_msgs::Pose>();
			BOOST_FOREACH(object_manipulation_msgs::Grasp grasp,res.grasps)
			{
				poses_msg.poses.push_back(grasp.grasp_pose);
			}
			_PosePublisher.publish(poses_msg);

			// publishing grasp contact plane
			tf::Transform transform = tf::Transform::getIdentity();
			object_manipulation_msgs::Grasp &grasp = res.grasps[0];
			tf::poseMsgToTF(grasp.grasp_pose,transform);
			geometry_msgs::PolygonStamped polygon_msg;
			polygon_msg.header.frame_id = worldFrameId;
			polygon_msg.header.stamp = ros::Time::now();
			polygon_msg.polygon = GraspPlannerServer::createPlanePolygon(transform,0.2f,0.2f);
			_PlanePublisher.publish(polygon_msg);
		}
		return success;
	}
	else
	{
		return false;
	}
}

geometry_msgs::Polygon GraspPlannerServer::createPlanePolygon(const tf::Transform &transform,tfScalar width, tfScalar length)
{
	geometry_msgs::Polygon polygon = geometry_msgs::Polygon();
	tf::Vector3 point;
	std::vector<tf::Vector3> planePoints = std::vector<tf::Vector3>();
	planePoints.push_back(transform(tf::Vector3(width/2,length/2,0)));
	planePoints.push_back(transform(tf::Vector3(width/2,-length/2,0)));
	planePoints.push_back(transform(tf::Vector3(-width/2,-length/2,0)));
	planePoints.push_back(transform(tf::Vector3(-width/2,length/2,0)));

	polygon.points = std::vector<geometry_msgs::Point32>();
	BOOST_FOREACH(tf::Vector3 v,planePoints)
	{
		geometry_msgs::Point32 p;
		p.x = v.x();
		p.y = v.y();
		p.z = v.z();
		polygon.points.push_back(p);
	}
	return polygon;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"grasp_planner_server");
	ros::NodeHandle nh;

	GraspPlannerServer graspPlannerServer = GraspPlannerServer();
	graspPlannerServer.init();

	return 0;
}
