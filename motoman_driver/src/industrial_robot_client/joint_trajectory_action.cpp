/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Fraunhofer IPA
 * Author: Thiago de Freitas
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Fraunhofer IPA, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#include <motoman_driver/industrial_robot_client/joint_trajectory_action.h>
#include "motoman_driver/industrial_robot_client/motoman_utils.h"
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

using industrial_robot_client::motoman_utils::getJointGroups;

namespace industrial_robot_client
{
namespace joint_trajectory_action
{

const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction::JointTrajectoryAction() :
  action_server_(node_, "joint_trajectory_action",
                 boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                 boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false)
{
  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  getJointGroups("topic_list", robot_groups_);

  for (size_t i = 0; i < robot_groups_.size(); i++)
  {
    std::vector<std::string> rg_joint_names = robot_groups_[i].get_joint_names();
    int group_number_int = robot_groups_[i].get_group_id();

    all_joint_names_.insert(all_joint_names_.end(), rg_joint_names.begin(), rg_joint_names.end());

    // Trajectories sub-group active map initialization
    has_active_goals_map_[group_number_int] = false;
    // Trajectory state recvd init
    trajectory_state_recvd_map_[group_number_int] = false;
  }

  pub_trajectory_command_ = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                              "joint_path_command", 1);

  sub_trajectory_state_  = node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
                              "feedback_states", 1,
                              boost::bind(&JointTrajectoryAction::controllerStateCB, this, _1));

  sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);

  // Set watchdog timer for entire robot state.
  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_),
                              boost::bind(&JointTrajectoryAction::watchdog, this, _1));

  has_active_goal_ = false;
  motion_started_ = false;

  action_server_.start();
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

void JointTrajectoryAction::robotStatusCB(
  const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg;  // caching robot status for later use.
  motion_started_ |= (msg->in_motion.val != industrial_msgs::TriState::FALSE);
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }
  bool trajectory_state_recvd = true;
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    trajectory_state_recvd &= trajectory_state_recvd_map_[group_number] | !has_active_goals_map_[group_number];
  }

  if (!trajectory_state_recvd)
  {
    ROS_DEBUG("Trajectory state of the entire robot not received since last watchdog");
    for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
    {
      if (!trajectory_state_recvd_map_[group_number])
      {
        ROS_DEBUG("Group [%s] state not received since last watchdog",
          robot_groups_[group_number].get_name().c_str());
      }
    }
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    if (!trajectory_state_recvd)
    {
      // last_trajectory_state_ is null if the subscriber never makes a connection
      if (!last_trajectory_state_)
      {
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM(
          "Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
      }
      abortGoal();
    }
  }

  // Reset the multi-group trajectory state received flag
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    trajectory_state_recvd_map_[group_number] = false;
  }
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_DEBUG("Multi-group trajectory execution request received");

  if (!gh.getGoal()->trajectory.points.empty())
  {
    std::map<int, size_t> group_joints_start_idx;
    ros::Duration last_time_from_start(0.0);

    if (has_active_goal_)  // Cancels the currently active goal.
    {
      ROS_WARN("Received new goal, canceling current goal");
      cancelGoal();
    }
    // Detect which robot groups are active in the current motion trajectory
    for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
    {
      size_t ros_idx = std::find(
                        gh.getGoal()->trajectory.joint_names.begin(),
                        gh.getGoal()->trajectory.joint_names.end(),
                        robot_groups_[group_number].get_joint_names()[0])
                      - gh.getGoal()->trajectory.joint_names.begin();

      bool is_found = ros_idx < gh.getGoal()->trajectory.joint_names.size();
      if (is_found)
      {
        has_active_goals_map_[group_number] = true;
        group_joints_start_idx.insert(std::pair<int, size_t>(group_number, ros_idx));
        ROS_DEBUG("Group [%s] is active in trajectory plan", robot_groups_[group_number].get_name().c_str());
      }
      else
      {
        has_active_goals_map_[group_number] = false;
        ROS_DEBUG("Group [%s] not present in trajectory plan, using its last received joint positions as goal",
                                                                robot_groups_[group_number].get_name().c_str());
      }
    }
    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    motoman_msgs::DynamicJointTrajectory dyn_traj;

    ROS_DEBUG("Publishing trajectory");

    current_traj_ = active_goal_.getGoal()->trajectory;

    for (int i = 0; i < gh.getGoal()->trajectory.points.size(); i++)
    {
      motoman_msgs::DynamicJointPoint dpoint;

      for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
      {
        motoman_msgs::DynamicJointsGroup dyn_group;

        int num_joints = robot_groups_[group_number].get_joint_names().size();

        if (has_active_goals_map_[group_number])  // If group is active on current goal
        {
          if (gh.getGoal()->trajectory.points[i].positions.empty())
          {
            std::vector<double> positions(num_joints, 0.0);
            dyn_group.positions = positions;
          }
          else
          {
            // This assumes joints in the same group are sequential and in the group-defined order.
            dyn_group.positions.insert(
              dyn_group.positions.begin(),
              gh.getGoal()->trajectory.points[i].positions.begin() + group_joints_start_idx[group_number],
              gh.getGoal()->trajectory.points[i].positions.begin() + group_joints_start_idx[group_number]
              + robot_groups_[group_number].get_joint_names().size());
          }

          if (gh.getGoal()->trajectory.points[i].velocities.empty())
          {
            std::vector<double> velocities(num_joints, 0.0);
            dyn_group.velocities = velocities;
          }
          else
          {
            dyn_group.velocities.insert(
              dyn_group.velocities.begin(),
              gh.getGoal()->trajectory.points[i].velocities.begin() + group_joints_start_idx[group_number],
              gh.getGoal()->trajectory.points[i].velocities.begin() + group_joints_start_idx[group_number]
              + robot_groups_[group_number].get_joint_names().size());
          }

          if (gh.getGoal()->trajectory.points[i].accelerations.empty())
          {
            std::vector<double> accelerations(num_joints, 0.0);
            dyn_group.accelerations = accelerations;
          }
          else
          {
            dyn_group.accelerations.insert(
              dyn_group.accelerations.begin(),
              gh.getGoal()->trajectory.points[i].accelerations.begin() + group_joints_start_idx[group_number],
              gh.getGoal()->trajectory.points[i].accelerations.begin() + group_joints_start_idx[group_number]
                + robot_groups_[group_number].get_joint_names().size());
          }

          if (gh.getGoal()->trajectory.points[i].effort.empty())
          {
            std::vector<double> effort(num_joints, 0.0);
            dyn_group.effort = effort;
          }
          else
          {
            dyn_group.effort.insert(
              dyn_group.effort.begin(),
              gh.getGoal()->trajectory.points[i].effort.begin() + group_joints_start_idx[group_number],
              gh.getGoal()->trajectory.points[i].effort.begin() + group_joints_start_idx[group_number]
                + robot_groups_[group_number].get_joint_names().size());
          }

          dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
          dyn_group.group_number = group_number;
          dyn_group.num_joints = dyn_group.positions.size();
        }

        // Generating message for groups that were not present in the trajectory message
        // Assume that joints from these groups will mantain its current position.
        // Velocity, acceleration and effort are zero out.
        else
        {
          std::vector<double> positions = last_trajectory_state_map_[group_number]->actual.positions;
          std::vector<double> velocities(num_joints, 0.0);
          std::vector<double> accelerations(num_joints, 0.0);
          std::vector<double> effort(num_joints, 0.0);

          dyn_group.positions = positions;
          dyn_group.velocities = velocities;
          dyn_group.accelerations = accelerations;
          dyn_group.effort = effort;

          dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
          dyn_group.group_number = group_number;
          dyn_group.num_joints = num_joints;
        }

        dpoint.groups.push_back(dyn_group);
      }

      dpoint.num_groups = dpoint.groups.size();
      dyn_traj.points.push_back(dpoint);
    }

    dyn_traj.header = gh.getGoal()->trajectory.header;
    dyn_traj.header.stamp = ros::Time::now();
    // Publishing the joint names for the 4 groups
    dyn_traj.joint_names = all_joint_names_;

    motion_started_ = false;
    this->pub_trajectory_command_.publish(dyn_traj);
  }
  else
  {
    ROS_ERROR("Multi-group joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM(
      "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_DEBUG("Received action cancel request");
  if (has_active_goal_ && active_goal_ == gh)
  {
    cancelGoal();
  }
  else
  {
    ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction::controllerStateCB(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG("Checking controller state feedback");

  // Full robot state is marked as received if all robot groups states have been received.
  int msg_group = -1;
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    size_t idx = std::find(msg->joint_names.begin(),
                           msg->joint_names.end(),
                           robot_groups_[group_number].get_joint_names()[0])
                          - msg->joint_names.begin();

    bool is_found = idx < msg->joint_names.size();
    if (is_found)
    {
      msg_group = group_number;
      break;
    }
  }

  if (msg_group == -1)
  {
    ROS_WARN("Unrecognized controller feedback message");
    return;
  }

  last_trajectory_state_ = msg;
  trajectory_state_recvd_map_[msg_group] = true;
  last_trajectory_state_map_[msg_group] = msg;

  bool trajectory_state_recvd = true;
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    trajectory_state_recvd &= trajectory_state_recvd_map_[group_number] | !has_active_goals_map_[group_number];
  }

  if (!trajectory_state_recvd)
  {
    ROS_DEBUG("Waiting for all robot groups feedback states before processing multi-group feedback");
    return;
  }
  if (!has_active_goal_)
  {
    ROS_DEBUG("No active multi-group goal, ignoring feedback");
    return;
  }

  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!motion_started_)
  {
    ROS_DEBUG("Motion has not started, ignoring feedback");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receivesjoint_traj_client
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        setGoalSuccess();
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        // TODO(NA): Should this really return succeeded?
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        setGoalSuccess();
      }
      else
      {
        ROS_DEBUG("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      setGoalSuccess();
    }
  }
  else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
  {
    ROS_WARN("Outside goal constraints, aborting trajectory");
    abortGoal();
  }
}

void JointTrajectoryAction::setGoalSuccess()
{
  ROS_INFO("Marking goal as successful.");

  // Marks the current goal as aborted.
  active_goal_.setSucceeded();
  has_active_goal_ = false;
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    has_active_goals_map_[group_number] = false;
  }
}

void JointTrajectoryAction::abortGoal()
{
  ROS_INFO("Aborting active goal.");
  // Stops the controller.
  motoman_msgs::DynamicJointTrajectory empty;
  pub_trajectory_command_.publish(empty);

  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    has_active_goals_map_[group_number] = false;
  }
}

void JointTrajectoryAction::cancelGoal()
{
  ROS_INFO("Cancelling active goal.");
  // Stops the controller.
  motoman_msgs::DynamicJointTrajectory empty;
  pub_trajectory_command_.publish(empty);

  // Marks the current goal as aborted.
  active_goal_.setCanceled();
  has_active_goal_ = false;
  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    has_active_goals_map_[group_number] = false;
  }
}

bool JointTrajectoryAction::withinGoalConstraints(const trajectory_msgs::JointTrajectory & traj)
{
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    return false;
  }

  // Assume true, and proof wrong by checking each robot group that is active in current goal
  std::map<int, bool> groups_at_goal_state_map;
  int last_point = traj.points.size() - 1;

  for (int group_number = 0; group_number < robot_groups_.size(); group_number++)
  {
    // Check if robot group is active in current multi-group trajectory goal.
    if (has_active_goals_map_[group_number])
    {
      ROS_DEBUG("Checking if group [%s] has reached its goal", robot_groups_[group_number].get_name().c_str());
      // Asume group is not at goal position
      groups_at_goal_state_map[group_number] = false;

      // Again this assumes that joints are defined in-order within their groups.
      size_t group_joints_start_idx = std::find(
                      traj.joint_names.begin(),
                      traj.joint_names.end(),
                      robot_groups_[group_number].get_joint_names()[0])
                    - traj.joint_names.begin();
      // Get an ordered map of the robot group last feedback state.
      std::map<std::string, double> robot_group_last_state_map;
      industrial_robot_client::utils::toMap(robot_groups_[group_number].get_joint_names(),
                                            last_trajectory_state_map_[group_number]->actual.positions,
                                            robot_group_last_state_map);
      // Get an ordered map of the robot group goal state.
      std::vector<double> group_goal_positions(
              traj.points[last_point].positions.begin() + group_joints_start_idx,
              traj.points[last_point].positions.begin() + group_joints_start_idx
                + robot_groups_[group_number].get_joint_names().size());
      std::map<std::string, double> group_traj_last_point_map;
      industrial_robot_client::utils::toMap(robot_groups_[group_number].get_joint_names(),
                                            group_goal_positions,
                                            group_traj_last_point_map);
      // Check if group is already at goal position
      if ( !industrial_robot_client::utils::isWithinRange(robot_groups_[group_number].get_joint_names(),
                                                        group_traj_last_point_map,
                                                        robot_group_last_state_map,
                                                        goal_threshold_) )
      {
        // Current Robot Group is not in goal position yet
        groups_at_goal_state_map[group_number] = false;
        return false;        // Stop checking other sub-groups
      }
      else
      {
        groups_at_goal_state_map[group_number] = true;
      }
    }
  }
  if (groups_at_goal_state_map.empty())
  {
    ROS_ERROR("Multi-group goal has not a single active group");
    return false;
  }
  // If this point is reached, all active groups are at goal position
  return true;
}

}  // namespace joint_trajectory_action
}  // namespace industrial_robot_client
