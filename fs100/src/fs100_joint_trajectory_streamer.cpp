/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#include "fs100/fs100_joint_trajectory_streamer.h"
#include "fs100/simple_message/motoman_motion_ctrl_message.h"
#include "fs100/simple_message/motoman_motion_reply_message.h"
#include "simple_message/messages/joint_traj_pt_full_message.h"
#include "industrial_utils/param_utils.h"
#include "industrial_utils/utils.h"

using namespace industrial::simple_message;
using namespace motoman::simple_message::motion_ctrl;
using namespace motoman::simple_message::motion_reply;
using industrial::joint_data::JointData;
using industrial::joint_traj_pt_full::JointTrajPtFull;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using motoman::simple_message::motion_ctrl_message::MotionCtrlMessage;
using motoman::simple_message::motion_reply_message::MotionReplyMessage;
namespace TransferStates = industrial_robot_client::joint_trajectory_streamer::TransferStates;

namespace motoman
{
namespace fs100_joint_trajectory_streamer
{

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

// override init() to read "robot_id" parameter and subscribe to joint_states
bool FS100_JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("FS100_JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryStreamer::init(connection, joint_names, velocity_limits);

  // try to read robot_id parameter, if none specified
  if ( (robot_id_ < 0) )
    node_.param("robot_id", robot_id_, 0);

  // try to read velocity limits from URDF, if none specified
  if (joint_vel_limits_.empty() && !industrial_utils::param::getJointVelocityLimits("robot_description", joint_vel_limits_))
    ROS_WARN("Unable to read velocity limits from 'robot_description' param.  Velocity validation disabled.");

  // set up joint_state subscriber, for trajectory validation
  sub_cur_pos_ = node_.subscribe("joint_states", 1,
                                 &FS100_JointTrajectoryStreamer::jointStateCB, this);

  return rtn;
}

FS100_JointTrajectoryStreamer::~FS100_JointTrajectoryStreamer()
{
  //TODO Find better place to call StopTrajMode
  setTrajMode(false);   // release TrajMode, so INFORM jobs can run
}

// override trajectory_to_msgs to provide validateTrajectory() check before sending any points
bool FS100_JointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr& traj, std::vector<SimpleMessage>* msgs)
{
  // check for valid trajectory point
  if (!validateTrajectory(*traj))
    return false;

  return JointTrajectoryStreamer::trajectory_to_msgs(traj, msgs);
}

// override create_message to generate JointTrajPtFull message (instead of default JointTrajPt)
bool FS100_JointTrajectoryStreamer::create_message(int seq, const trajectory_msgs::JointTrajectoryPoint &pt, SimpleMessage *msg)
{
  JointTrajPtFull msg_data;
  JointData values;

  // copy position data
  if (!pt.positions.empty())
  {
    if (VectorToJointData(pt.positions, values))
      msg_data.setPositions(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy position data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearPositions();

  // copy velocity data
  if (!pt.velocities.empty())
  {
    if (VectorToJointData(pt.velocities, values))
      msg_data.setVelocities(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy velocity data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearVelocities();

  // copy acceleration data
  if (!pt.accelerations.empty())
  {
    if (VectorToJointData(pt.accelerations, values))
      msg_data.setAccelerations(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy acceleration data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearAccelerations();

  // copy scalar data
  msg_data.setRobotID(robot_id_);
  msg_data.setSequence(seq);
  msg_data.setTime(pt.time_from_start.toSec());

  // convert to message
  JointTrajPtFullMessage jtpf_msg;
  jtpf_msg.init(msg_data);

  return jtpf_msg.toRequest(*msg);  // assume "request" COMM_TYPE for now
}

bool FS100_JointTrajectoryStreamer::VectorToJointData(const std::vector<double> &vec,
                                                      JointData &joints)
{
  if ( vec.size() > joints.getMaxNumJoints() )
    ROS_ERROR_RETURN(false, "Failed to copy to JointData.  Len (%d) out of range (0 to %d)",
                     vec.size(), joints.getMaxNumJoints());

  joints.init();
  for (int i=0; i<vec.size(); ++i)
    joints.setJoint(i, vec[i]);

  return true;
}

// override send_to_robot to provide controllerReady() and setTrajMode() calls
bool FS100_JointTrajectoryStreamer::send_to_robot(const std::vector<SimpleMessage>& messages)
{
  if (!controllerReady() && !setTrajMode(true))
    ROS_ERROR_RETURN(false, "Failed to initialize MotoRos motion.  Trajectory ABORTED.  Correct issue and re-send trajectory.");

  return JointTrajectoryStreamer::send_to_robot(messages);
}

// override streamingThread, to provide check/retry of MotionReply.result=BUSY
void FS100_JointTrajectoryStreamer::streamingThread()
{
  int connectRetryCount = 1;

  ROS_INFO("Starting FS100 joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

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
        if (this->current_point_ >= (int)this->current_traj_.size())
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
        if (!this->connection_->sendAndReceiveMsg(msg, reply, false))
          ROS_WARN("Failed sent joint point, will try again");
        else
        {
          MotionReplyMessage reply_status;
          if (!reply_status.init(reply))
          {
            ROS_ERROR("Aborting trajectory: Unable to parse JointTrajectoryPoint reply");
            this->state_ = TransferStates::IDLE;
            break;
          }

          if (reply_status.reply_.getResult() == MotionReplyResults::SUCCESS)
          {
            ROS_DEBUG("Point[%d of %d] sent to controller",
                     this->current_point_, (int)this->current_traj_.size());
            this->current_point_++;
          }
          else if (reply_status.reply_.getResult() == MotionReplyResults::BUSY)
            break;  // silently retry sending this point
          else
          {
            ROS_ERROR_STREAM("Aborting Trajectory.  Failed to send point: " << reply_status.reply_.getResultString());
            this->state_ = TransferStates::IDLE;
            break;
          }
        }

        break;
      default:
        ROS_ERROR("Joint trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }

    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

// override trajectoryStop to send MotionCtrl message
void FS100_JointTrajectoryStreamer::trajectoryStop()
{
  MotionReply reply;

  this->state_ = TransferStates::IDLE;  // stop sending trajectory points

  if (!sendMotionCtrlMsg(MotionControlCmds::STOP_MOTION, reply))
    return;

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to Stop Motion: " << getErrorString(reply));
    return;
  }
}

bool FS100_JointTrajectoryStreamer::validateTrajectory(const trajectory_msgs::JointTrajectory &traj)
{
  for (int i=0; i<traj.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[i];

    if (pt.positions.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing position data for trajectory pt %d", i);

    if (pt.velocities.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing velocity data for trajectory pt %d", i);

    for (int j=0; j<pt.velocities.size(); ++j)
    {
      std::map<std::string, double>::iterator max_vel = joint_vel_limits_.find(traj.joint_names[j]);
      if (max_vel == joint_vel_limits_.end()) continue;  // no velocity-checking if limit not defined

      if (std::abs(pt.velocities[j]) > max_vel->second)
        ROS_ERROR_RETURN(false, "Validation failed: Max velocity exceeded for trajectory pt %d, joint '%s'", i, traj.joint_names[j].c_str());
    }

    if ((i > 0) && (pt.time_from_start.toSec() == 0))
      ROS_ERROR_RETURN(false, "Validation failed: Missing valid timestamp data for trajectory pt %d", i);
  }

  if ((cur_pos_.header.stamp - ros::Time::now()).toSec() > pos_stale_time_)
    ROS_ERROR_RETURN(false, "Validation failed: Can't get current robot position.");

  sensor_msgs::JointState start_pos;
  start_pos.name = traj.joint_names;
  start_pos.position = traj.points[0].positions;
  ROS_DEBUG_STREAM("cur_pos: " << cur_pos_);
  ROS_DEBUG_STREAM("start_pos: " << start_pos);
  if (!industrial_utils::isSimilar(cur_pos_, start_pos, start_pos_tol_))
    ROS_ERROR_RETURN(false, "Validation failed: Trajectory doesn't start at current position.");

  return true;
}

// copy robot JointState into local cache
void FS100_JointTrajectoryStreamer::jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
  this->cur_pos_ = *msg;
}

bool FS100_JointTrajectoryStreamer::sendMotionCtrlMsg(MotionControlCmd command, MotionReply &reply)
{
  SimpleMessage req, res;
  MotionCtrl data;
  MotionCtrlMessage ctrl_msg;
  MotionReplyMessage ctrl_reply;

  data.init(robot_id_, 0, command, 0);
  ctrl_msg.init(data);
  ctrl_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, res))
    ROS_ERROR_RETURN(false, "Failed to send MotionCtrl message");

  ctrl_reply.init(res);
  reply.copyFrom(ctrl_reply.reply_);

  return true;
}

bool FS100_JointTrajectoryStreamer::controllerReady()
{
  std::string err_str;
  MotionReply reply;

  if (!sendMotionCtrlMsg(MotionControlCmds::CHECK_MOTION_READY, reply))
    return false;

 return (reply.getResult() == MotionReplyResults::TRUE);
}

bool FS100_JointTrajectoryStreamer::setTrajMode(bool enable)
{
  MotionReply reply;
  MotionControlCmd cmd = enable ? MotionControlCmds::START_TRAJ_MODE : MotionControlCmds::STOP_TRAJ_MODE;

  if (!sendMotionCtrlMsg(cmd, reply))
    return false;

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to set TrajectoryMode: " << getErrorString(reply));
    return false;
  }

  return true;
}

std::string FS100_JointTrajectoryStreamer::getErrorString(const MotionReply &reply)
{
  std::ostringstream ss;
  ss << reply.getResultString() << " (" << reply.getResult() << ")";
  ss << " : ";
  ss << reply.getSubcodeString() << " (" << reply.getSubcode() << ")";
  return ss.str();
}


} //fs100_joint_trajectory_streamer
} //industrial_robot_client

