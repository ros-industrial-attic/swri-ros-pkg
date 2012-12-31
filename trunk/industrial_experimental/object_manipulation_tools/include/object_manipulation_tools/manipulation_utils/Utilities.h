
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

	// post multiplies a rotation to actual_pose such that the z vector points in the direction of that in
	// rectification_pose.  No translation is produced.
	void rectifyPoseZDirection(const geometry_msgs::Pose &actual_pose,
			const geometry_msgs::Pose &rectification_pose,geometry_msgs::Pose &rectified_pose);
	void rectifyPoseZDirection(const geometry_msgs::Pose &actual_pose,
			const tf::Transform &rectification_tf,geometry_msgs::Pose &rectified_pose);
}

#endif
