/*
 * Utilities.cpp
 *
 *  Created on: Dec 25, 2012
 *      Author: coky
 */
#include <object_manipulation_tools/manipulation_utils/Utilities.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/centroid.h>

namespace manipulation_utils
{
/*
 * generateGraspPoses
 * Description:
 * 		Generates additional poses by post multiplying a rotation matrix to the original pose.  The rotation matrix is
 * 		produced by rotating about a axis by an angle increment.
 *
 * Input arguments:
 * 		pose:			Original pose.
 * 		rotationAxix:	Axis of rotation.
 * 		numCandidates:	Number of poses to generate.  The angle increment will be calculated  as 2*pi/numCandidates.
 *
 * 	Output arguments:
 * 		poses:			Array of generated poses
 */
	void generateGraspPoses(const geometry_msgs::Pose &pose,tf::Vector3 rotationAxis,int numCandidates,
			std::vector<geometry_msgs::Pose> &poses)
	{
		tf::Transform graspTf = tf::Transform::getIdentity();
		tf::Transform candidateTf;
		tfScalar angle = tfScalar(2 * M_PI/(double(numCandidates)));

		// converting initial pose to tf
		tf::poseMsgToTF(pose,graspTf);

		for(int i = 0; i < numCandidates; i++)
		{
			candidateTf = graspTf*tf::Transform(tf::Quaternion(rotationAxis,i*angle - M_PI),
					tf::Vector3(0.0f,0.0f,0.0f));
			geometry_msgs::Pose candidatePose = geometry_msgs::Pose();
			tf::poseTFToMsg(candidateTf,candidatePose);
			poses.push_back(candidatePose);
		}
	}

	/*
	 * Generates additional grasps by rotating original grasp about the rotational axis 'numCandidate' times
	 */
	void generateCandidateGrasps(const object_manipulation_msgs::Grasp firstGrasp,tf::Vector3 rotationAxis,int numCandidates,
			std::vector<object_manipulation_msgs::Grasp> &graspCandidates)
	{
		std::vector<geometry_msgs::Pose> graspPoses;
		generateGraspPoses(firstGrasp.grasp_pose,rotationAxis,numCandidates,graspPoses);

		// filling candidate grasp array
		object_manipulation_msgs::Grasp grasp = firstGrasp;
		for(std::size_t i = 0; i < graspPoses.size(); i++)
		{
			grasp.grasp_pose = graspPoses[i];
			graspCandidates.push_back(grasp);
		}
	}

	void createBoundingSphereCollisionModel(const sensor_msgs::PointCloud cluster,double radius,
			arm_navigation_msgs::CollisionObject &obj)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		sensor_msgs::PointCloud2 clusterMsg;
		sensor_msgs::convertPointCloudToPointCloud2(cluster,clusterMsg);
		pcl::fromROSMsg(clusterMsg,cloud);

		// computing centroid
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(cloud,centroid);

		// saving centroid as in tf
		tf::Transform t = tf::Transform::getIdentity();
		t.setOrigin(tf::Vector3(centroid[0],centroid[1],centroid[2]));


		// creating shape and pose
		geometry_msgs::Pose pose;
		arm_navigation_msgs::Shape shape;
		shape.type = arm_navigation_msgs::Shape::SPHERE;
		shape.dimensions.push_back(radius);
		tf::poseTFToMsg(t,pose);

		// storing results in object
		obj.shapes.push_back(shape);
		obj.poses.push_back(pose);
	}
}
