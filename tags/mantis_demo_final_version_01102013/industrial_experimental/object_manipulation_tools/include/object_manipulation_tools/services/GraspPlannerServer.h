/*
 * GraspPlannerServer.h
 *
 *  Created on: Aug 9, 2012
 *      Author: jnicho
 */

#ifndef GRASPPLANNERSERVER_H_
#define GRASPPLANNERSERVER_H_

#include <ros/ros.h>
#include <object_manipulator/object_manipulator.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <ros/package.h>
#include <object_manipulation_tools/grasp_planners/OverheadGraspPlanner.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <boost/shared_ptr.hpp>

const std::string SERVICE_NAME = "plan_point_cluster_grasp";
const std::string POSE_PUBLISH_TOPIC_NAME = "grasp_poses";
const std::string PLANE_PUBLISH_TOPIC_NAME = "contact_plane";
const std::string PARAM_NAME_PUBLISH_RESULTS = "publish_results";

class GraspPlannerServer {
public:
	GraspPlannerServer(boost::shared_ptr<GraspPlannerInterface> &plannerPtr);
	virtual ~GraspPlannerServer();
	void init();
	void finish();
	bool serviceCallback(object_manipulation_msgs::GraspPlanning::Request &request,
			object_manipulation_msgs::GraspPlanning::Response &response);

	void timerCallback(const ros::TimerEvent &evnt);

	static geometry_msgs::Polygon createPlanePolygon(const tf::Transform &transform,
			tfScalar width = 0.1, tfScalar length = 0.1);

protected:

	boost::shared_ptr<GraspPlannerInterface> _GraspPlanner;
	ros::ServiceServer _ServiceServer;
	ros::Publisher _PosePublisher;
	ros::Publisher _PlanePublisher;
	ros::Timer _PublishTimer;
	double _PublishInterval;

	std::string _PosePubTopic;
	std::string _PlanePubTopic;
	std::string _ServiceName;

	// from ros parameter server
	bool _PublishResults;

	// last set of valid messages
	bool _ResultsSet;
	geometry_msgs::PolygonStamped _LastValidTablePolygonMsg;
	geometry_msgs::PoseStamped _LastValidGraspPoseMsg;
};

#endif /* GRASPPLANNERSERVER_H_ */
