
#ifndef MANIPULATION_UTILS_H_
#define MANIPULATION_UTILS_H_
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <object_manipulation_msgs/Grasp.h>
#include <arm_navigation_msgs/CollisionObject.h>

namespace manipulation_utils
{
	void generateGraspPoses(const geometry_msgs::Pose &pose,tf::Vector3 rotationAxis,int numCandidates,
			std::vector<geometry_msgs::Pose> &poses);

	/*
	 * Generates additional grasps by rotating original grasp about the rotational axis 'numCandidate' times
	 */
	void generateCandidateGrasps(const object_manipulation_msgs::Grasp firstGrasp,tf::Vector3 rotationAxis,int numCandidates,
			std::vector<object_manipulation_msgs::Grasp> &graspCandidates);

	void createBoundingSphereCollisionModel(const sensor_msgs::PointCloud cluster,double radius,
			arm_navigation_msgs::CollisionObject &obj);
}

#endif
