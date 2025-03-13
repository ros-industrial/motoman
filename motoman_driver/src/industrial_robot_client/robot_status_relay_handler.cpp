/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#include "motoman_driver/industrial_robot_client/robot_status_relay_handler.h"
#include "industrial_msgs/RobotStatus.h"
#include "simple_message/log_wrapper.h"

namespace industrial_robot_client
{
namespace robot_status_relay_handler
{

using industrial::simple_message::SimpleMessage;
using industrial::simple_message::StandardMsgTypes::StandardMsgType;
using industrial::simple_message::CommTypes::CommType;
using industrial::simple_message::ReplyTypes::ReplyType;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::robot_status::RobotModes::RobotMode;
using industrial::robot_status::TriStates::TriState;
using industrial::robot_status::TriStates::toROSMsgEnum;
using industrial::robot_status_message::RobotStatusMessage;

bool RobotStatusRelayHandler::init(SmplMsgConnection* connection)
{
  m_auto_enable_robot = true;
  m_serviceRobotAutoEnable = this->node_.advertiseService("robot_auto_enable",
    &RobotStatusRelayHandler::robotAutoEnableCB, this);

  ROS_INFO("[RobotStatusRelayHandler] advertising robot_auto_enable");

  this->pub_robot_status_ = this->node_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);

  return init(static_cast<int>(StandardMsgType::STATUS), connection);
}

bool RobotStatusRelayHandler::internalCB(SimpleMessage& in)
{
  RobotStatusMessage status_msg;

  if (!status_msg.init(in))
  {
    LOG_ERROR("Failed to initialize status message");
    return false;
  }

  return internalCB(status_msg);
}

bool RobotStatusRelayHandler::robotAutoEnableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  m_auto_enable_robot = req.data;
  res.success = true;
  ROS_INFO("[RobotStatusRelayHandler] auto enable robot: %d", m_auto_enable_robot);
  return true;
}

bool RobotStatusRelayHandler::internalCB(RobotStatusMessage & in)
{
  industrial_msgs::RobotStatus status;
  bool rtn = true;

  status.header.stamp = ros::Time::now();
  status.drives_powered.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getDrivesPowered());
  status.e_stopped.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getEStopped());
  status.error_code = in.status_.getErrorCode();
  status.in_error.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getInError());
  status.in_motion.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getInMotion());
  status.mode.val = industrial::robot_status::RobotModes::toROSMsgEnum(in.status_.getMode());
  if (!m_auto_enable_robot && (status.motion_possible.val == TriState::TS_FALSE))
  {
    // override motion_possible value to avoid RobotStatus Error if we wanted to disconnect from the robot
    status.motion_possible.val = TriState::TS_UNKNOWN;
  }
  else
  {
    status.motion_possible.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getMotionPossible());
  }

  this->pub_robot_status_.publish(status);

  // Reply back to the controller if the sender requested it.
  if (CommType::SERVICE_REQUEST == in.getCommType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyType::SUCCESS : ReplyType::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  ros::spinOnce();  // handle service call if requested

  return rtn;
}

}  // namespace robot_status_relay_handler
}  // namespace industrial_robot_client
