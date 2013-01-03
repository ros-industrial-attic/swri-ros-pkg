/*
 * GraspPlannerServer.cpp
 *
 *  Created on: Aug 9, 2012
 *      Author: jnicho
 */

#include "object_manipulation_tools/services/GraspPlannerServer.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <boost/foreach.hpp>

GraspPlannerServer::GraspPlannerServer(boost::shared_ptr<GraspPlannerInterface> &plannerPtr):
_GraspPlanner(plannerPtr),
_ServiceName(SERVICE_NAME),
_PublishResults(false),
_PosePubTopic(POSE_PUBLISH_TOPIC_NAME),
_PlanePubTopic(PLANE_PUBLISH_TOPIC_NAME),
_PublishInterval(0.4f),
_ResultsSet(false)
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

	//checking pointer to grasp planner
	if(_GraspPlanner == NULL)
	{
		ROS_ERROR_STREAM("Grasp Planner Server: Grasp planner pointer is null, exiting");
		ros::shutdown();
		return;
	}

	// fetching parameters from ros
	std::string nameSpace = "/" + ros::this_node::getName();
	ros::param::param(nameSpace + "/" + PARAM_NAME_PUBLISH_RESULTS,_PublishResults,false);

	// setting up ros server
	_ServiceServer = nh.advertiseService(_ServiceName,&GraspPlannerServer::serviceCallback,this);
	ROS_INFO("%s",std::string(_GraspPlanner->getPlannerName() + " advertising service: " + _ServiceName).c_str());

	// setting up ros timer
	_PublishTimer = nh.createTimer(ros::Duration(_PublishInterval),&GraspPlannerServer::timerCallback,this);

	// setting up ros publishers
	_PlanePublisher = nh.advertise<geometry_msgs::PolygonStamped>(nameSpace + "/" + _PlanePubTopic,1);
	_PosePublisher = nh.advertise<geometry_msgs::PoseStamped>(nameSpace + "/" + _PosePubTopic,1);
	ros::spin();
}

void GraspPlannerServer::finish()
{
//	if(_GraspPlanner)
//	{
//		delete _GraspPlanner;
//		_GraspPlanner = NULL;
//	}
}

bool GraspPlannerServer::serviceCallback(object_manipulation_msgs::GraspPlanning::Request &req,
		object_manipulation_msgs::GraspPlanning::Response &res)
{
	if(_GraspPlanner)
	{
		bool success = _GraspPlanner->planGrasp(req,res);

		if(success)
		{
			std::string worldFrameId = req.target.reference_frame_id;

			// storing last valid grasp pose
			_LastValidGraspPoseMsg.header.frame_id = worldFrameId;
			_LastValidGraspPoseMsg.pose = geometry_msgs::Pose();

			if(res.grasps.size() > 0)
			{
				_LastValidGraspPoseMsg.pose = res.grasps[0].grasp_pose;
			}

			// storing last valid contact plane grasp contact plane
			tf::Transform transform = tf::Transform::getIdentity();
			tf::poseMsgToTF(res.grasps[0].grasp_pose,transform);
			_LastValidTablePolygonMsg.header.frame_id = worldFrameId;
			_LastValidTablePolygonMsg.polygon = GraspPlannerServer::createPlanePolygon(transform,0.2f,0.2f);

			_ResultsSet = true;
		}

		return success;
	}
	else
	{
		return false;
	}
}

void GraspPlannerServer::timerCallback(const ros::TimerEvent &evnt)
{
	// retrieving parameter from ros
	std::string nameSpace = "/" + ros::this_node::getName();
	ros::param::param(nameSpace + "/" + PARAM_NAME_PUBLISH_RESULTS,_PublishResults,_PublishResults);

	if(_PublishResults && _ResultsSet)
	{
		ros::Time pubTime = ros::Time::now();
		_LastValidGraspPoseMsg.header.stamp = pubTime;
		_LastValidTablePolygonMsg.header.stamp = pubTime;

		_PosePublisher.publish(_LastValidGraspPoseMsg);
		_PlanePublisher.publish(_LastValidTablePolygonMsg);
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

//int main(int argc, char** argv)
//{
//	ros::init(argc,argv,"grasp_planner_server");
//	ros::NodeHandle nh;
//
//	GraspPlannerServer graspPlannerServer = GraspPlannerServer();
//	graspPlannerServer.init();
//
//	return 0;
//}
