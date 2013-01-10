/*
 * GraspSequenceValidator.h
 *
 *  Created on: Oct 31, 2012
 *      Author: coky
 */

#ifndef GRASPSEQUENCEVALIDATOR_H_
#define GRASPSEQUENCEVALIDATOR_H_


#include "object_manipulator/grasp_execution/approach_lift_grasp.h"
#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>
#include <pluginlib/class_loader.h>
#include <object_manipulator/grasp_execution/grasp_tester_fast.h>

//! Uses an interpolated IK approach from pregrasp to final grasp
/*! Initial check consists of IK for the pre-grasp, followed by generation
  of an interpolated IK path from pre-grasp to grasp. This is then followed
  by computation of an interpolated IK path from grasp to lift.

  Execution consists of using move arm to go to the pre-grasp, then execution of
  the interpolated IK paths for grasp. Lift is executed separately, called from
  the higher level executor.

  Note that we do not use the pre-grasp from the database directly; rather, the
  pre-grasp is obtained by backing up the grasp by a pre-defined amount. This
  allows us more flexibility in choosing the pre-grasp. However, we can no longer
  always assume the pre-grasp is collision free, which we could if we used the
  pre-grasp from the database directly.

  In the most recent database version, the pre-grasp was obtained by backing up
  10 cm, so we know at least that that is not colliding with the object.
*/

class GraspSequenceValidator : public object_manipulator::GraspTester
{
public:
	struct GraspSequenceDetails: public object_manipulator::GraspExecutionInfo
	{
	public:

		trajectory_msgs::JointTrajectory twist_trajectory_; // should be executed after the approach trajectory
	};

protected:

  //! Dynamic link padding to be used for grasp operation
  virtual std::vector<arm_navigation_msgs::LinkPadding>
    linkPaddingForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal);

  bool getInterpolatedIK(const std::string& arm_name,
                         const tf::Transform& first_pose,
                         const tf::Vector3& direction,
                         const double& distance,
                         const std::vector<double>& ik_solution,
                         const bool& reverse,
                         const bool& premultiply,
                         trajectory_msgs::JointTrajectory& traj);

  bool getInterpolatedIK(const std::string& arm_name,
          const tf::Transform& initialTransform,
          const tf::Quaternion &rot,
          const std::vector<double>& ikSolutionAtStart,
          std::vector<double>& ikSolutionAtEnd,
          trajectory_msgs::JointTrajectory& traj);

  std::map<std::string, arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*> ik_solver_map_;
  double consistent_angle_;
  unsigned int num_points_;
  unsigned int redundancy_;

  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;

  planning_environment::CollisionModels* getCollisionModels();
  planning_models::KinematicState* getPlanningSceneState();

  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* state_;

 public:

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;

  //! Also adds a grasp marker at the pre-grasp location
  GraspSequenceValidator(planning_environment::CollisionModels* cm = NULL,
		  const std::string& plugin_name="");

  ~GraspSequenceValidator();

  void setPlanningSceneState(planning_models::KinematicState* state)
  {
    state_ = state;
  }

  void getGroupJoints(const std::string& group_name,
                      std::vector<std::string>& group_links);

  void getGroupLinks(const std::string& group_name,
                     std::vector<std::string>& group_links);

  virtual void testGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
          const object_manipulation_msgs::Grasp &grasp,
          object_manipulator::GraspExecutionInfo &execution_info)
  {


  }

  virtual void testGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<object_manipulation_msgs::Grasp> &grasps,
                          std::vector<object_manipulator::GraspExecutionInfo> &execution_info,
                          bool return_on_first_hit);

  virtual void testGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<object_manipulation_msgs::Grasp> &grasps,double twistAngle,
                          std::vector<GraspSequenceDetails> &execution_info,
                          bool return_on_first_hit);
};


#endif /* GRASPSEQUENCEVALIDATOR_H_ */
