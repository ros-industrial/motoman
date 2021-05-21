/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
 *  * Neither the name of the Southwest Research Institute, nor the names
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

#include "motoman_driver/industrial_robot_client/joint_trajectory_streamer.h"
#include <algorithm>
#include <queue>
#include <map>
#include <vector>
#include <string>

namespace CommTypes = industrial::simple_message::CommTypes;
namespace ReplyTypes = industrial::simple_message::ReplyTypes;

namespace industrial_robot_client
{
namespace joint_trajectory_streamer
{

bool JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::map<int, RobotGroup> &robot_groups,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, robot_groups, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
    new boost::thread(boost::bind(&JointTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

bool JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
    new boost::thread(boost::bind(&JointTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

JointTrajectoryStreamer::~JointTrajectoryStreamer()
{
  delete this->streaming_thread_;
}

void JointTrajectoryStreamer::jointTrajectoryCB(const motoman_msgs::DynamicJointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  std::vector<SimpleMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

void JointTrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  std::vector<SimpleMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

void JointTrajectoryStreamer::jointCommandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);

  // Point specific variables that can persist between IDLE and POINT_STREAM state
  bool start_point = false;

  const size_t num_msg_points = msg->points.size();

  trajectory_msgs::JointTrajectoryPoint rbt_pt;

  //If current state is idle, set to POINT_STREAMING
  if (TransferStates::IDLE == state)
  {
    // Check to see if the message contains more than one trajectory point, currently the
    // POINT_STREAMING state only accepts a single point
    if (num_msg_points != 1)
    {
      ROS_ERROR("JointTrajectory command must contain a single point, ignoring message and maintaining IDLE state");
      return;
    }

    // Get the message point and select
    const trajectory_msgs::JointTrajectoryPoint msg_pt = msg->points[0];

    if (!select(msg->joint_names, msg_pt, this->all_joint_names_, &rbt_pt))
    {
      // Select function will report message to console, just return here to stay in IDLE state
      return;
    }
    else
    {
      // Check for required zero velocities
      const double zero_tolerance = 1e-5;
      bool zero_velocities = true;

      for (size_t i = 0; i < rbt_pt.velocities.size(); ++i)
      {
        if (std::abs(rbt_pt.velocities[i]) > zero_tolerance )
        {
          zero_velocities = false;
          break;
        }
      }

      if (!zero_velocities)
      {
        ROS_ERROR("Starting joint point must contain zero velocity for each joint,"
                  " unable to transition to on-the-fly streaming");
        return;
      }
    }

    // Have a valid starting point, update the flag and the time from start variable
    // Note: last_time_from_start_ variable is only used in this function therefore we do not need to wrap in lock
    start_point = true;
    ptstreaming_last_time_from_start_ = msg_pt.time_from_start;

    // Update point streaming sequence count, empty the point queue and set internal state to point streaming
    this->mutex_.lock();
    this->state_ = TransferStates::POINT_STREAMING;
    this->ptstreaming_seq_count_ = 0;
    this->ptstreaming_queue_ = std::queue<SimpleMessage>();
    this->mutex_.unlock();

    // Set the local state to point streaming to force enqueuing of the starting point
    state = TransferStates::POINT_STREAMING;

    ROS_INFO("Starting joint point received. Starting on-the-fly streaming.");
  }

  // Process incoming point since we are in point streaming mode
  if (TransferStates::POINT_STREAMING == state)
  {
    bool stop_trajectory = false;

    ros::Duration pt_time_from_start;

    // Empty points is a trigger to abort the POINT_STREAMING mode by stopping the trajectory
    if (msg->points.empty())
    {
      ROS_INFO("Empty point received");
      stop_trajectory = true;
    }
    else if (num_msg_points != 1)  // Check for invalid number of trajectory points
    {
      ROS_ERROR("JointTrajectory command must contain a single point");
      stop_trajectory = true;
    }
    else  // We have at one point, let check for timing if we are not the start point
    {
      pt_time_from_start = msg->points[0].time_from_start;
      if (!start_point && (pt_time_from_start <= ptstreaming_last_time_from_start_))
      {
        ROS_ERROR("JointTrajectory point must have a time from start duration that is greater than the previously "
                  "processes point");
        stop_trajectory = true;
      }
    }

    if (ptstreaming_queue_.size() > max_ptstreaming_queue_elements)  // Check for max queue size
    {
      ROS_ERROR("Point streaming queue has reached max allowed elements");
      stop_trajectory = true;
    }

    // Stop the trajectory and cancel the point streaming mode if we need to
    if (stop_trajectory)
    {
      ROS_INFO("Canceling on-the-fly streaming");
      this->mutex_.lock();
      trajectoryStop();
      this->mutex_.unlock();
      return;
    }

    // select / reorder joints for sending to robot (if it is not the starting point)
    if (!start_point)
    {
      if (!select(msg->joint_names, msg->points[0], this->all_joint_names_, &rbt_pt))
      {
        return;
      }
    }

    trajectory_msgs::JointTrajectoryPoint xform_pt;
    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return;

    SimpleMessage message;
    // convert trajectory point to ROS message
    if (!create_message(this->ptstreaming_seq_count_, xform_pt, &message))
      return;

    // Update the last time from start
    // Note: last_time_from_start_ variable is only used in this function therefore we do not need to wrap in lock
    ptstreaming_last_time_from_start_ = pt_time_from_start;

    // Points get pushed into queue here. They will be popped in the Streaming Thread and sent to controller.
    this->mutex_.lock();
    this->ptstreaming_queue_.push(message);
    this->ptstreaming_seq_count_++;
    this->mutex_.unlock();
  }

  //Else, cannot splice. Cancel current motion.
  else
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }
}

bool JointTrajectoryStreamer::send_to_robot(const std::vector<SimpleMessage>& messages)
{
  ROS_INFO("Loading trajectory, setting state to streaming");
  this->mutex_.lock();
  {
    ROS_INFO("Executing trajectory of size: %d", static_cast<int>(messages.size()));
    this->current_traj_ = messages;
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->streaming_start_ = ros::Time::now();
  }
  this->mutex_.unlock();

  return true;
}

bool JointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj,
                                                 std::vector<SimpleMessage>* msgs)
{
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", static_cast<int>(msgs->size()), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}

bool JointTrajectoryStreamer::trajectory_to_msgs(const motoman_msgs::DynamicJointTrajectoryConstPtr& traj,
                                                 std::vector<SimpleMessage>* msgs)
{
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", static_cast<int>(msgs->size()), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}


void JointTrajectoryStreamer::streamingThread()
{
  int connectRetryCount = 1;

  ROS_INFO("Starting joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.001).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, tmpMsg, reply;

    switch (this->state_)
    {
    case TransferStates::IDLE:
      ros::Duration(0.250).sleep();  //  slower loop while waiting for new trajectory
      break;

    case TransferStates::STREAMING:
      if (this->current_point_ >= static_cast<int>(this->current_traj_.size()))
      {
        ROS_INFO("Trajectory streaming complete, setting state to IDLE");
        this->state_ = TransferStates::IDLE;
        break;
      }

      if (!this->connection_->isConnected())
      {
        ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
        connectRetryCount = 5;
        break;
      }

      tmpMsg = this->current_traj_[this->current_point_];
      msg.init(tmpMsg.getMessageType(), CommTypes::SERVICE_REQUEST,
               ReplyTypes::INVALID, tmpMsg.getData());  // set commType=REQUEST

      ROS_DEBUG("Sending joint trajectory point");
      if (this->connection_->sendAndReceiveMsg(msg, reply, false))
      {
        ROS_INFO("Point[%d of %d] sent to controller",
                 this->current_point_, static_cast<int>(this->current_traj_.size()));
        this->current_point_++;
      }
      else
        ROS_WARN("Failed sent joint point, will try again");

      break;

    case TransferStates::POINT_STREAMING:

      // if no points in queue, streaming complete, set to idle.
      if (this->ptstreaming_queue_.empty())
      {
        ROS_INFO("Point streaming complete, setting state to IDLE");
        this->state_ = TransferStates::IDLE;
        break;
      }
      // if not connected, reconnect.
      if (!this->connection_->isConnected())
      {
        ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
        connectRetryCount = 5;
        break;
      }
      // otherwise, send point to robot.
      tmpMsg = this->ptstreaming_queue_.front();
      this->ptstreaming_queue_.pop();
      msg.init(tmpMsg.getMessageType(), CommTypes::SERVICE_REQUEST,
               ReplyTypes::INVALID, tmpMsg.getData());  // set commType=REQUEST

      ROS_DEBUG("Sending joint trajectory point");
      if (this->connection_->sendAndReceiveMsg(msg, reply, false))
      {
        ROS_INFO("Point[%d] sent to controller", this->current_point_);
        this->current_point_++;
      }
      else
        ROS_WARN("Failed sent joint point, will try again");

      break;
      // TODO Consider checking for controller point starvation here. use a
      //      timer to check if the state is popping in and out of
      //      POINT_STREAMING mode, indicating something is trying to send
      //      streaming points, but is doing so too slowly. It may, in fact, not
      //      matter other than motion won't be smooth.

    default:
      ROS_ERROR("Joint trajectory streamer: unknown state, %d", this->state_);
      this->state_ = TransferStates::IDLE;
      break;
    }

    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

void JointTrajectoryStreamer::trajectoryStop()
{
  JointTrajectoryInterface::trajectoryStop();

  ROS_DEBUG("Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;
}

}  // namespace joint_trajectory_streamer
}  // namespace industrial_robot_client

