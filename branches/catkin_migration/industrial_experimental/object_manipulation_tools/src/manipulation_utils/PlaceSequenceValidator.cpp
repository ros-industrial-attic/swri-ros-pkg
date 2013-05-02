/*
 * PlaceSequenceValidator.cpp
 *
 *  Created on: Oct 10, 2012
 */

/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <object_manipulation_tools/manipulation_utils/PlaceSequenceValidator.h>
#include <tf/transform_listener.h>
using namespace object_manipulator;
using namespace object_manipulation_msgs;

PlaceSequenceValidator::PlaceSequenceValidator()
{

}

PlaceSequenceValidator::PlaceSequenceValidator(planning_environment::CollisionModels* cm,
		  const std::string& plugin_name)
:object_manipulator::PlaceTesterFast(cm,plugin_name),
 _TcpToWristTransform(tf::Transform::getIdentity())
{
	// TODO Auto-generated constructor stub

}

PlaceSequenceValidator::~PlaceSequenceValidator() {
	// TODO Auto-generated destructor stub
}

void PlaceSequenceValidator::setTcpToWristTransform(const tf::Transform &tcpTransform)
{
	_TcpToWristTransform = tcpTransform;
}

void PlaceSequenceValidator::testPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
        const std::vector<geometry_msgs::PoseStamped> &place_locations,
        std::vector<object_manipulator::PlaceExecutionInfo> &execution_info,
        bool return_on_first_hit)
{
  ros::WallTime start = ros::WallTime::now();

  std::map<unsigned int, unsigned int> outcome_count;

  planning_environment::CollisionModels* cm = getCollisionModels();
  planning_models::KinematicState* state = getPlanningSceneState();

  std::map<std::string, double> planning_scene_state_values;
  state->getKinematicStateValues(planning_scene_state_values);

  std::vector<std::string> end_effector_links, arm_links;
  getGroupLinks(handDescription().gripperCollisionName(place_goal.arm_name), end_effector_links);
  getGroupLinks(handDescription().armGroup(place_goal.arm_name), arm_links);

  collision_space::EnvironmentModel::AllowedCollisionMatrix original_acm = cm->getCurrentAllowedCollisionMatrix();
  cm->disableCollisionsForNonUpdatedLinks(place_goal.arm_name);
  collision_space::EnvironmentModel::AllowedCollisionMatrix group_disable_acm = cm->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_disable_acm = group_disable_acm;
  if(!place_goal.collision_support_surface_name.empty()) {
    if(place_goal.collision_support_surface_name == "\"all\"")
    {
      object_disable_acm.changeEntry(place_goal.collision_object_name, true);
    }
    else
    {
      object_disable_acm.changeEntry(place_goal.collision_object_name, place_goal.collision_support_surface_name, true);
    }
  }
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_support_disable_acm = object_disable_acm;
  if(place_goal.allow_gripper_support_collision) {
    if(place_goal.collision_support_surface_name == "\"all\"")
    {
      for(unsigned int i = 0; i < end_effector_links.size(); i++)
      {
    	  object_support_disable_acm.changeEntry(end_effector_links[i], true);
      }
    }
    else
    {
      object_support_disable_acm.changeEntry(place_goal.collision_support_surface_name, end_effector_links, true);
    }
  }
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_all_arm_disable_acm = object_disable_acm;
  collision_space::EnvironmentModel::AllowedCollisionMatrix object_support_all_arm_disable_acm = object_support_disable_acm;
  collision_space::EnvironmentModel::AllowedCollisionMatrix group_all_arm_disable_acm = group_disable_acm;

  //turning off collisions for the arm associated with this end effector
  for(unsigned int i = 0; i < arm_links.size(); i++) {
    object_all_arm_disable_acm.changeEntry(arm_links[i], true);
    object_support_all_arm_disable_acm.changeEntry(arm_links[i], true);
    group_all_arm_disable_acm.changeEntry(arm_links[i], true);
  }
  cm->setAlteredAllowedCollisionMatrix(object_support_all_arm_disable_acm);

  //first we apply link padding for place check
  cm->applyLinkPaddingToCollisionSpace(linkPaddingForPlace(place_goal));

  execution_info.clear();
  execution_info.resize(place_locations.size());

  tf::Vector3 approach_dir;
  tf::vector3MsgToTF(doNegate(place_goal.approach.direction.vector), approach_dir);
  approach_dir.normalize();
  tf::Vector3 distance_approach_dir = approach_dir*fabs(place_goal.approach.desired_distance);
  tf::Transform approach_trans(tf::Quaternion(0,0,0,1.0), distance_approach_dir);

  tf::Vector3 retreat_dir;
  tf::vector3MsgToTF(doNegate(handDescription().approachDirection(place_goal.arm_name)), retreat_dir);
  retreat_dir.normalize();
  tf::Vector3 distance_retreat_dir = retreat_dir*fabs(place_goal.desired_retreat_distance);
  tf::Transform retreat_trans(tf::Quaternion(0,0,0,1.0), distance_retreat_dir);

  std::map<std::string, double> planning_scene_state_values_post_grasp = planning_scene_state_values_post_grasp;
  std::map<std::string, double> post_grasp_joint_vals;
  std::map<std::string, double> grasp_joint_vals;
  for(unsigned int j = 0; j < place_goal.grasp.pre_grasp_posture.name.size(); j++)
  {
    planning_scene_state_values_post_grasp[place_goal.grasp.pre_grasp_posture.name[j]] = place_goal.grasp.pre_grasp_posture.position[j];
    post_grasp_joint_vals[place_goal.grasp.pre_grasp_posture.name[j]] = place_goal.grasp.pre_grasp_posture.position[j];
    grasp_joint_vals[place_goal.grasp.pre_grasp_posture.name[j]] = planning_scene_state_values[place_goal.grasp.pre_grasp_posture.name[j]];
  }

  // store all gripper poses relative to the world
  std::vector<tf::Transform> gripperTcpInWorldPoses(place_locations.size());

  //now this is place specific
  for(unsigned int i = 0; i < place_locations.size(); i++)
  {
    //using the grasp posture
    state->setKinematicState(post_grasp_joint_vals);

    //always true
    execution_info[i].result_.continuation_possible = true;

    if(!cm->convertPoseGivenWorldTransform(*state,
                                           cm->getWorldFrameId(),
                                           place_locations[i].header,
                                           place_locations[i].pose,
                                           execution_info[i].gripper_place_pose_))
    {
      ROS_INFO_STREAM("Something wrong with pose conversion");
      continue;
    }
    tf::poseMsgToTF(execution_info[i].gripper_place_pose_.pose, gripperTcpInWorldPoses[i]);
    tf::Transform gripperInObjPose; // wrist frame relative to object
    tf::poseMsgToTF(place_goal.grasp.grasp_pose, gripperInObjPose);

    //post multiply for object frame in order to obtain gripper pose in world coordinates
    gripperTcpInWorldPoses[i] = gripperTcpInWorldPoses[i]*gripperInObjPose;


    tf::poseTFToMsg(gripperTcpInWorldPoses[i], execution_info[i].gripper_place_pose_.pose);
    state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(place_goal.arm_name),gripperTcpInWorldPoses[i]);

    if(cm->isKinematicStateInCollision(*state))
    {
      ROS_INFO_STREAM("gripper at place move is in collision");
      execution_info[i].result_.result_code = PlaceLocationResult::PLACE_IN_COLLISION;
      outcome_count[PlaceLocationResult::PLACE_IN_COLLISION]++;
    }
    else
    {
    	//ROS_INFO_STREAM("gripper at place move is collision free");
        execution_info[i].result_.result_code = 0;
    }
  }

  //now we revert link paddings for approach and retreat checks
  cm->revertCollisionSpacePaddingToDefault();

  //now we do the place approach pose, not allowing anything different
  cm->setAlteredAllowedCollisionMatrix(group_all_arm_disable_acm);

  for(unsigned int i = 0; i < place_locations.size(); i++)
  {

    if(execution_info[i].result_.result_code != 0) continue;

    state->setKinematicState(planning_scene_state_values);

    tf::Transform approach_pose = approach_trans*gripperTcpInWorldPoses[i];
    state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(place_goal.arm_name),approach_pose);

    if(cm->isKinematicStateInCollision(*state))
    {
      ROS_INFO_STREAM("gripper at pre-place is in collision");

      execution_info[i].result_.result_code = PlaceLocationResult::PREPLACE_IN_COLLISION;
      outcome_count[PlaceLocationResult::PREPLACE_IN_COLLISION]++;
      continue;
    }
    else
    {
    	//ROS_INFO_STREAM("gripper at pre-place move is collision free");
    }
  }

  //TODO - for now, just have to hope that we're not in contact with the object
  //after we release, but there's not a good way to check this for now
  //so we just leave the object all arm disable on for the retreat position check
  cm->setAlteredAllowedCollisionMatrix(object_all_arm_disable_acm);

  //first we do retreat, with the gripper at post grasp position
  for(unsigned int i = 0; i < place_locations.size(); i++) {

    if(execution_info[i].result_.result_code != 0) continue;

    state->setKinematicState(post_grasp_joint_vals);

    tf::Transform retreat_pose = gripperTcpInWorldPoses[i]*retreat_trans;
    state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(place_goal.arm_name),retreat_pose);

    if(cm->isKinematicStateInCollision(*state))
    {
      /*
	std_msgs::ColorRGBA col_pregrasp;
	col_pregrasp.r = 0.0;
	col_pregrasp.g = 1.0;
	col_pregrasp.b = 1.0;
	col_pregrasp.a = 1.0;
	visualization_msgs::MarkerArray arr;
	cm_->getRobotMarkersGivenState(*state, arr, col_pregrasp,
				       "retreat",
				       ros::Duration(0.0),
				       &end_effector_links);
	vis_marker_array_publisher_.publish(arr);
      */

      ROS_INFO_STREAM("gripper at retreat move is in collision");
      execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_IN_COLLISION;
      outcome_count[PlaceLocationResult::RETREAT_IN_COLLISION]++;

    }
    else
    {
    	//ROS_INFO_STREAM("gripper at retreat move is collision free");
    }
  }

  std_msgs::Header world_header;
  world_header.frame_id = cm->getWorldFrameId();
  const std::vector<std::string>& joint_names = ik_solver_map_[place_goal.arm_name]->getJointNames();

  if(return_on_first_hit)
  {

    bool last_ik_failed = false;
    for(unsigned int i = 0; i < place_locations.size(); i++)
    {

      if(execution_info[i].result_.result_code != 0) continue;

      if(!last_ik_failed)
      {
        //now we move to the ik portion, which requires re-enabling collisions for the arms
        cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

        //and also reducing link paddings
        cm->applyLinkPaddingToCollisionSpace(linkPaddingForPlace(place_goal));
      }
      //getting back to original state for seed
      state->setKinematicState(planning_scene_state_values_post_grasp);

      //now call ik for grasp
      geometry_msgs::Pose place_geom_pose; // pose of the wrist relative to the world
      tf::poseTFToMsg(gripperTcpInWorldPoses[i]*_TcpToWristTransform, place_geom_pose);

      geometry_msgs::PoseStamped wristInArmBasePose; // pose of the wrist relative to the arm base
      cm->convertPoseGivenWorldTransform(*state,
					 ik_solver_map_[place_goal.arm_name]->getBaseName(),
                                         world_header,
                                         place_geom_pose,
                                         wristInArmBasePose);

      arm_navigation_msgs::Constraints emp;
      sensor_msgs::JointState solution;
      arm_navigation_msgs::ArmNavigationErrorCodes error_code;
      if(!ik_solver_map_[place_goal.arm_name]->findConstraintAwareSolution(wristInArmBasePose.pose,
                                                                           emp,
                                                                           state,
                                                                           solution,
                                                                           error_code,
                                                                           false))
      {
        ROS_INFO_STREAM("Place out of reach, skipping to next candidate");
        execution_info[i].result_.result_code = PlaceLocationResult::PLACE_OUT_OF_REACH;
        outcome_count[PlaceLocationResult::PLACE_OUT_OF_REACH]++;
        last_ik_failed = true;
        continue;
      }
      else
      {
    	ROS_INFO_STREAM("Place move is in reach");
        last_ik_failed = false;
      }

      state->setKinematicState(grasp_joint_vals);

      //now we solve interpolated ik
      tf::Transform wristInArmBaseTransform;
      tf::poseMsgToTF(wristInArmBasePose.pose, wristInArmBaseTransform);
      //now we need to do interpolated ik
      execution_info[i].descend_trajectory_.joint_names = joint_names;

      if(!getInterpolatedIK(place_goal.arm_name,
                            wristInArmBaseTransform,
                            approach_dir,
                            place_goal.approach.desired_distance,
                            solution.position,
                            true,
                            true,
                            execution_info[i].descend_trajectory_))
      {
        ROS_INFO_STREAM("No interpolated IK for approach to place");
        execution_info[i].result_.result_code = PlaceLocationResult::PLACE_UNFEASIBLE;
        outcome_count[PlaceLocationResult::PLACE_UNFEASIBLE]++;
        continue;
      }
      else
      {
    	  ROS_INFO_STREAM("Interpolated IK for approach move found");
      }

      state->setKinematicState(planning_scene_state_values_post_grasp);
      execution_info[i].retreat_trajectory_.joint_names = joint_names;
      if(!getInterpolatedIK(place_goal.arm_name,
                            wristInArmBaseTransform,
                            retreat_dir,
                            place_goal.desired_retreat_distance,
                            solution.position,
                            false,
                            true,
                            execution_info[i].retreat_trajectory_))
      {
        ROS_INFO_STREAM("No interpolated IK for place to retreat");
        execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_UNFEASIBLE;
        outcome_count[PlaceLocationResult::RETREAT_UNFEASIBLE]++;
        continue;
      }
      else
      {
    	  ROS_INFO_STREAM("Interpolated IK for place move found");
      }

      //now we revert link paddings and object collisions and do a final check for the initial ik points
      cm->revertCollisionSpacePaddingToDefault();

      //the start of the place approach needs to be collision-free according to the default collision matrix
      cm->setAlteredAllowedCollisionMatrix(group_disable_acm);

      if(execution_info[i].descend_trajectory_.points.empty()) {
        ROS_WARN_STREAM("No result code and no points in approach trajectory");
        continue;
      }

      std::map<std::string, double> pre_place_ik = grasp_joint_vals;
      for(unsigned int j = 0; j < joint_names.size(); j++)
      {
        pre_place_ik[joint_names[j]] = execution_info[i].descend_trajectory_.points[0].positions[j];
      }


      state->setKinematicState(pre_place_ik);
      if(cm->isKinematicStateInCollision(*state))
      {
        ROS_INFO_STREAM("Final collision pre-place check failed");
        execution_info[i].result_.result_code = PlaceLocationResult::PREPLACE_OUT_OF_REACH;
        outcome_count[PlaceLocationResult::PREPLACE_OUT_OF_REACH]++;
        continue;
      }

      //the end of the place retreat also needs to be collision-free according to the default collision matrix
      if(execution_info[i].retreat_trajectory_.points.empty())
      {
        ROS_WARN_STREAM("No result code and no points in retreat trajectory");
        continue;
      }

      std::map<std::string, double> retreat_ik = post_grasp_joint_vals;
      for(unsigned int j = 0; j < joint_names.size(); j++)
      {
       retreat_ik[joint_names[j]] = execution_info[i].retreat_trajectory_.points.back().positions[j];
      }
      state->setKinematicState(retreat_ik);
      if(cm->isKinematicStateInCollision(*state))
      {
        ROS_INFO_STREAM("Final collision retreat check failed");
        execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_OUT_OF_REACH;
        outcome_count[PlaceLocationResult::RETREAT_OUT_OF_REACH]++;
        continue;
      }
      else
      {
        ROS_INFO_STREAM("Everything successful");
        execution_info[i].result_.result_code = PlaceLocationResult::SUCCESS;
	execution_info.resize(i+1);
        outcome_count[PlaceLocationResult::SUCCESS]++;
        break;
      }
    }

    for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
        it != outcome_count.end();
        it++)
    {
      ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
    }
    cm->setAlteredAllowedCollisionMatrix(original_acm);
    return;
  }

  //now we move to the ik portion, which requires re-enabling collisions for the arms
  cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

  //and also reducing link paddings
  cm->applyLinkPaddingToCollisionSpace(linkPaddingForPlace(place_goal));

  for(unsigned int i = 0; i < place_locations.size(); i++)
  {

    if(execution_info[i].result_.result_code != 0) continue;

    //getting back to original state for seed
    state->setKinematicState(planning_scene_state_values_post_grasp);

    //now call ik for grasp
    geometry_msgs::Pose place_geom_pose;
    tf::poseTFToMsg(gripperTcpInWorldPoses[i]*_TcpToWristTransform, place_geom_pose);
    geometry_msgs::PoseStamped wristInArmBasePose;
    cm->convertPoseGivenWorldTransform(*state,
				       ik_solver_map_[place_goal.arm_name]->getBaseName(),
                                       world_header,
                                       place_geom_pose,
                                       wristInArmBasePose);

    arm_navigation_msgs::Constraints emp;
    sensor_msgs::JointState solution;
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    if(!ik_solver_map_[place_goal.arm_name]->findConstraintAwareSolution(wristInArmBasePose.pose,
                                                                         emp,
                                                                         state,
                                                                         solution,
                                                                         error_code,
                                                                         false))
    {
      ROS_INFO_STREAM("Place out of reach");
      execution_info[i].result_.result_code = PlaceLocationResult::PLACE_OUT_OF_REACH;
      outcome_count[PlaceLocationResult::PLACE_OUT_OF_REACH]++;
      continue;
    }

    state->setKinematicState(grasp_joint_vals);

    //now we solve interpolated ik
    tf::Transform wristInArmBaseTransform;
    tf::poseMsgToTF(wristInArmBasePose.pose, wristInArmBaseTransform);
    //now we need to do interpolated ik
    execution_info[i].descend_trajectory_.joint_names = joint_names;
    if(!getInterpolatedIK(place_goal.arm_name,
                          wristInArmBaseTransform,
                          approach_dir,
                          place_goal.approach.desired_distance,
                          solution.position,
                          true,
                          true,
                          execution_info[i].descend_trajectory_)) {
      ROS_INFO_STREAM("No interpolated IK for approach to place");
      execution_info[i].result_.result_code = PlaceLocationResult::PLACE_UNFEASIBLE;
      outcome_count[PlaceLocationResult::PLACE_UNFEASIBLE]++;
      continue;
    }

    state->setKinematicState(planning_scene_state_values_post_grasp);
    execution_info[i].retreat_trajectory_.joint_names = joint_names;
    if(!getInterpolatedIK(place_goal.arm_name,
                          wristInArmBaseTransform,
                          retreat_dir,
                          place_goal.desired_retreat_distance,
                          solution.position,
                          false,
                          false,
                          execution_info[i].retreat_trajectory_))
    {
      ROS_INFO_STREAM("No interpolated IK for place to retreat");
      execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_UNFEASIBLE;
      outcome_count[PlaceLocationResult::RETREAT_UNFEASIBLE]++;
      continue;
    }
  }

  //now we revert link paddings and object collisions and do a final check for the initial ik points
  cm->revertCollisionSpacePaddingToDefault();

  cm->setAlteredAllowedCollisionMatrix(group_disable_acm);

  for(unsigned int i = 0; i < place_locations.size(); i++)
  {

    if(execution_info[i].result_.result_code != 0) continue;

    if(execution_info[i].descend_trajectory_.points.empty())
    {
      ROS_WARN_STREAM("No result code and no points in approach trajectory");
      continue;
    }

    std::map<std::string, double> pre_place_ik = grasp_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++)
    {
      pre_place_ik[joint_names[j]] = execution_info[i].descend_trajectory_.points[0].positions[j];
    }

    state->setKinematicState(pre_place_ik);
    if(cm->isKinematicStateInCollision(*state))
    {
      ROS_INFO_STREAM("Final pre-place check failed");
      execution_info[i].result_.result_code = PlaceLocationResult::PREPLACE_OUT_OF_REACH;
      outcome_count[PlaceLocationResult::PREPLACE_OUT_OF_REACH]++;
      continue;
    }

  }

  //now we need to disable collisions with the object for lift
  cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

  for(unsigned int i = 0; i < place_locations.size(); i++)
  {

    if(execution_info[i].result_.result_code != 0) continue;

    if(execution_info[i].retreat_trajectory_.points.empty())
    {
      ROS_WARN_STREAM("No result code and no points in retreat trajectory");
      continue;
    }

    std::map<std::string, double> retreat_ik = post_grasp_joint_vals;
    for(unsigned int j = 0; j < joint_names.size(); j++)
    {
      retreat_ik[joint_names[j]] = execution_info[i].retreat_trajectory_.points.back().positions[j];
    }

    state->setKinematicState(retreat_ik);
    if(cm->isKinematicStateInCollision(*state))
    {
      ROS_INFO_STREAM("Final lift check failed");
      execution_info[i].result_.result_code = PlaceLocationResult::RETREAT_OUT_OF_REACH;
      outcome_count[PlaceLocationResult::RETREAT_OUT_OF_REACH]++;
      continue;
    }
    else
    {
      ROS_INFO_STREAM("Everything successful");
      execution_info[i].result_.result_code = PlaceLocationResult::SUCCESS;
      outcome_count[PlaceLocationResult::SUCCESS]++;
    }
  }
  cm->setAlteredAllowedCollisionMatrix(original_acm);

  ROS_INFO_STREAM("Took " << (ros::WallTime::now()-start).toSec());

  for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
      it != outcome_count.end();
      it++) {
    ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
  }
}
