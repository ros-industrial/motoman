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

using namespace motoman::simple_message::motion_ctrl;
using namespace motoman::simple_message::motion_reply;
using industrial::joint_data::JointData;
using industrial::joint_traj_pt_full::JointTrajPtFull;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using motoman::simple_message::motion_ctrl_message::MotionCtrlMessage;
using motoman::simple_message::motion_reply_message::MotionReplyMessage;

namespace motoman
{
namespace fs100_joint_trajectory_streamer
{

bool FS100_JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  //TODO: implement FS100_JointTrajectoryStreamer::init
  bool rtn = true;

  ROS_INFO("FS100_JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryStreamer::init(connection, joint_names, velocity_limits);

  return rtn;
}

FS100_JointTrajectoryStreamer::~FS100_JointTrajectoryStreamer()
{
  //TODO Find better place to call StopTrajMode
  setTrajMode(false);   // release TrajMode, so INFORM jobs can run
}

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
    {
      ROS_ERROR("Failed to copy position data to JointTrajPtFullMessage");
      return false;
    }
  }
  else
    msg_data.clearPositions();

  // copy velocity data
  if (!pt.velocities.empty())
  {
    if (VectorToJointData(pt.velocities, values))
      msg_data.setVelocities(values);
    else
    {
      ROS_ERROR("Failed to copy velocity data to JointTrajPtFullMessage");
      return false;
    }
  }
  else
    msg_data.clearVelocities();

  // copy acceleration data
  if (!pt.accelerations.empty())
  {
    if (VectorToJointData(pt.accelerations, values))
      msg_data.setAccelerations(values);
    else
    {
      ROS_ERROR("Failed to copy acceleration data to JointTrajPtFullMessage");
      return false;
    }
  }
  else
    msg_data.clearAccelerations();

  // copy scalar data
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
  {
    ROS_ERROR("Failed to copy to JointData.  Len (%d) out of range (0 to %d)",
              vec.size(), joints.getMaxNumJoints());
    return false;
  }

  joints.init();
  for (int i=0; i<vec.size(); ++i)
    joints.setJoint(i, vec[i]);

  return true;
}

bool FS100_JointTrajectoryStreamer::send_to_robot(const std::vector<SimpleMessage>& messages)
{
  if (!controllerReady() && !setTrajMode(true))
  {
    ROS_ERROR("Failed to initialize MotoRos motion.  Trajectory ABORTED.  Correct issue and re-send trajectory.");
    return false;
  }

  return JointTrajectoryStreamer::send_to_robot(messages);
}

void FS100_JointTrajectoryStreamer::trajectoryStop()
{
  MotionReply reply;

  if (!sendMotionCtrlMsg(MotionControlCmds::STOP_MOTION, reply))
    return;

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to Stop Motion: " << getErrorString(reply));
    return;
  }
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
  {
    ROS_ERROR("Failed to send MotionCtrl message");
    return false;
  }

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

