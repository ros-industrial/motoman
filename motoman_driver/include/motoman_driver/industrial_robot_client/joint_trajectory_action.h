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

#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_TRAJECTORY_ACTION_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_TRAJECTORY_ACTION_H

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>
#include <motoman_driver/industrial_robot_client/robot_group.h>
#include <motoman_driver/industrial_robot_client/joint_trajectory_action_v0.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_msgs/MotionReplyResult.h>
namespace industrial_robot_client
{
namespace joint_trajectory_action
{

class JointTrajectoryAction : public JointTrajectoryActionV0
{
public:
  /**
   * \brief Constructor
   *
   */
  JointTrajectoryAction();

  /**
   * \brief Destructor
   *
   */
  virtual ~JointTrajectoryAction();

private:

  std::map<int, ros::Subscriber> sub_motion_replies_;

  std::map<int, ros::Publisher> pub_trajectories_;

  std::map<int, RobotGroup> robot_groups_;

  std::map<int, ros::Subscriber> sub_trajectories_;

  std::map<int, ros::Subscriber> sub_status_;

  std::map<int, JointTractoryActionServer*> act_servers_;

  std::map<int, ros::Timer>watchdog_timer_map_;

  std::map<int, bool> has_active_goal_map_;

  std::map<int, JointTractoryActionServer::GoalHandle> active_goal_map_;

  std::map<int, trajectory_msgs::JointTrajectory> current_traj_map_;

  std::vector<std::string> all_joint_names_;

  std::map<int, control_msgs::FollowJointTrajectoryFeedbackConstPtr> last_trajectory_state_map_;

  /**
   * \brief Indicates trajectory state has been received.  Used by
   * watchdog to determine if the robot driver is responding.
   */
  bool trajectory_state_recvd_;

  std::map<int, bool> trajectory_state_recvd_map_;

  /**
   * \brief Watch dog callback, used to detect robot driver failures
   *
   * \param e time event information
   *
   */
  void watchdog(const ros::TimerEvent &e) override;

  void watchdog(const ros::TimerEvent &e, int group_number);

  /**
   * \brief Action server goal callback method
   *
   * \param gh goal handle
   *
   */
  void goalCB(JointTractoryActionServer::GoalHandle gh) override;

  /**
   * \brief Action server cancel callback method
   *
   * \param gh goal handle
   *
   */
  void cancelCB(JointTractoryActionServer::GoalHandle gh) override;

  /**
   * \brief Controller state callback (executed when feedback message
   * received)
   *
   * \param msg joint trajectory feedback message
   *
   */
  void goalCB(JointTractoryActionServer::GoalHandle gh, int group_number);

  /**
   * \brief Action server cancel callback method
   *
   * \param gh goal handle
   *
   */
  void cancelCB(JointTractoryActionServer::GoalHandle gh, int group_number);

  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);

  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, int robot_id);
  using JointTrajectoryActionV0::motionReplyCB;
  void motionReplyCB(const motoman_msgs::MotionReplyResultConstPtr &msg, int robot_id);
  using JointTrajectoryActionV0::abortGoal;
  void abortGoal(int robot_id);

  bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                             const trajectory_msgs::JointTrajectory & traj, int robot_id);
};

}  // namespace joint_trajectory_action
}  // namespace industrial_robot_client

#endif  // MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_TRAJECTORY_ACTION_H

