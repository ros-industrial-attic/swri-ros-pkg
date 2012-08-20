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
#include <planners/OverheadGraspPlanner.h>
#include <geometry_msgs/Polygon.h>

const std::string SERVICE_NAME = "plan_point_cluster_grasp";
const std::string POSE_PUBLISH_TOPIC_NAME = "grasp_poses";
const std::string PLANE_PUBLISH_TOPIC_NAME = "contact_plane";
const std::string PARAM_NAME_PUBLISH_RESULTS = "publish_results";

class GraspPlannerServer {
public:
	GraspPlannerServer();
	virtual ~GraspPlannerServer();
	void init();
	void finish();
	bool serviceCallback(object_manipulation_msgs::GraspPlanning::Request &request,
			object_manipulation_msgs::GraspPlanning::Response &response);

	static geometry_msgs::Polygon createPlanePolygon(const tf::Transform &transform,
			tfScalar width = 0.1, tfScalar length = 0.1);

protected:

	GraspPlannerInterface *_GraspPlanner;
	ros::ServiceServer _ServiceServer;
	ros::Publisher _PosePublisher;
	ros::Publisher _PlanePublisher;

	std::string _PosePubTopic;
	std::string _PlanePubTopic;
	std::string _ServiceName;

	// from ros parameter server
	bool _PublishResults;
};

#endif /* GRASPPLANNERSERVER_H_ */
