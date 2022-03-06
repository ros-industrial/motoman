/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Delft Robotics Institute
 * Copyright (c) 2020, Southwest Research Institute
 * Copyright (c) 2021, Delft Robotics Institute
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

#include "motoman_driver/io_relay.h"
#include <string>
#include <cstdint>
#include <limits>
#include <ros/ros.h>
#include <sstream>

namespace motoman
{
namespace io_relay
{

using industrial::shared_types::shared_int;

bool MotomanIORelay::init(int default_port)
{
  std::string ip;
  int port;

  // override port with ROS param, if available
  const std::string port_param_name = "~port";
  // TODO( ): should really use a private NodeHandle here
  if (!ros::param::param<int>(port_param_name, port, default_port))
  {
    ROS_WARN_STREAM_NAMED("io.init", "Failed to get '" << port_param_name
      << "' parameter: using default (" << default_port << ")");
  }
  if (port < 0 || port > std::numeric_limits<uint16_t>::max())
  {
    ROS_FATAL_STREAM_NAMED("io.init", "Invalid value for port (" << port << "), "
      "must be between 0 and " << std::numeric_limits<uint16_t>::max() << ".");
    return false;
  }

  const std::string robot_ip_param_name = "robot_ip_address";
  if (!ros::param::get(robot_ip_param_name, ip) || ip.empty())
  {
    ROS_FATAL_STREAM_NAMED("io.init", "Invalid IP address: please set the '"
      << robot_ip_param_name << "' parameter");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  if (!default_tcp_connection_.init(ip_addr, port))
  {
    ROS_FATAL_NAMED("io.init", "Failed to initialize TcpClient");
    return false;
  }
  free(ip_addr);

  ROS_DEBUG_STREAM_NAMED("io.init", "I/O relay attempting to connect to: tcp://" << ip << ":" << port);
  if (!default_tcp_connection_.makeConnect())
  {
    ROS_FATAL_NAMED("io.init", "Failed to connect");
    return false;
  }

  if (!io_ctrl_.init(&default_tcp_connection_))
  {
    ROS_FATAL_NAMED("io.init", "Failed to initialize MotomanIoCtrl");
    return false;
  }

  this->srv_read_mregister = this->node_.advertiseService("read_mregister",
      &MotomanIORelay::readMRegisterCB, this);
  this->srv_read_single_io = this->node_.advertiseService("read_single_io",
      &MotomanIORelay::readSingleIoCB, this);
  this->srv_read_group_io = this->node_.advertiseService("read_group_io",
      &MotomanIORelay::readGroupIoCB, this);
  this->srv_write_mregister = this->node_.advertiseService("write_mregister",
      &MotomanIORelay::writeMRegisterCB, this);
  this->srv_write_single_io = this->node_.advertiseService("write_single_io",
      &MotomanIORelay::writeSingleIoCB, this);
  this->srv_write_group_io = this->node_.advertiseService("write_group_io",
      &MotomanIORelay::writeGroupIoCB, this);

  return true;
}

// Service to read an M register
bool MotomanIORelay::readMRegisterCB(
  motoman_msgs::ReadMRegister::Request &req,
  motoman_msgs::ReadMRegister::Response &res)
{
  shared_int io_val = -1;
  std::string err_msg;

  // send message and release mutex as soon as possible
  this->mutex_.lock();
  bool result = io_ctrl_.readMRegister(req.address, io_val, err_msg);
  this->mutex_.unlock();

  if (!result)
  {
    res.success = false;

    // provide caller with failure indication
    // TODO( ): should we also return the result code?
    std::stringstream message;
    message << "Read failed (address: " << req.address << "): " << err_msg;
    res.message = message.str();
    ROS_ERROR_STREAM_NAMED("io.read", res.message);

    return true;
  }

  ROS_DEBUG_STREAM_NAMED("io.read", "Address " << req.address << ", value: " << io_val);

  // no failure, so no need for an additional message
  res.value = io_val;
  res.success = true;
  return true;
}

// Service to read a single IO
bool MotomanIORelay::readSingleIoCB(
  motoman_msgs::ReadSingleIO::Request &req,
  motoman_msgs::ReadSingleIO::Response &res)
{
  shared_int io_val = -1;
  std::string err_msg;

  // send message and release mutex as soon as possible
  this->mutex_.lock();
  bool result = io_ctrl_.readSingleIO(req.address, io_val, err_msg);
  this->mutex_.unlock();

  if (!result)
  {
    res.success = false;

    // provide caller with failure indication
    // TODO( ): should we also return the result code?
    std::stringstream message;
    message << "Read failed (address: " << req.address << "): " << err_msg;
    res.message = message.str();
    ROS_ERROR_STREAM_NAMED("io.read", res.message);

    return true;
  }

  ROS_DEBUG_STREAM_NAMED("io.read", "Address " << req.address << ", value: " << io_val);

  // no failure, so no need for an additional message
  res.value = io_val;
  res.success = true;
  return true;
}

// Service to read a group IO
bool MotomanIORelay::readGroupIoCB(
  motoman_msgs::ReadGroupIO::Request &req,
  motoman_msgs::ReadGroupIO::Response &res)
{
  shared_int io_val = -1;
  std::string err_msg;

  // send message and release mutex as soon as possible
  this->mutex_.lock();
  bool result = io_ctrl_.readGroupIO(req.address, io_val, err_msg);
  this->mutex_.unlock();

  if (!result)
  {
    res.success = false;

    // provide caller with failure indication
    // TODO( ): should we also return the result code?
    std::stringstream message;
    message << "Group read failed (address: " << req.address << "): " << err_msg;
    res.message = message.str();
    ROS_ERROR_STREAM_NAMED("io.read", res.message);

    return true;
  }

  ROS_DEBUG_STREAM_NAMED("io.read", "Address " << req.address << ", value: " << io_val);

  // no failure, so no need for an additional message
  res.value = io_val;
  res.success = true;
  return true;
}

// Service to write an M register
bool MotomanIORelay::writeMRegisterCB(
  motoman_msgs::WriteMRegister::Request &req,
  motoman_msgs::WriteMRegister::Response &res)
{
  std::string err_msg;

  // send message and release mutex as soon as possible
  this->mutex_.lock();
  bool result = io_ctrl_.writeMRegister(req.address, req.value, err_msg);
  this->mutex_.unlock();

  if (!result)
  {
    res.success = false;

    // provide caller with failure indication
    // TODO( ): should we also return the result code?
    std::stringstream message;
    message << "Write failed (address: " << req.address << "): " << err_msg;
    res.message = message.str();
    ROS_ERROR_STREAM_NAMED("io.write", res.message);

    return true;
  }

  ROS_DEBUG_STREAM_NAMED("io.write", "Element " << req.address << " set to: " << req.value);

  // no failure, so no need for an additional message
  res.success = true;
  return true;
}

// Service to write Single IO
bool MotomanIORelay::writeSingleIoCB(
  motoman_msgs::WriteSingleIO::Request &req,
  motoman_msgs::WriteSingleIO::Response &res)
{
  std::string err_msg;

  // send message and release mutex as soon as possible
  this->mutex_.lock();
  bool result = io_ctrl_.writeSingleIO(req.address, req.value, err_msg);
  this->mutex_.unlock();

  if (!result)
  {
    res.success = false;

    // provide caller with failure indication
    // TODO( ): should we also return the result code?
    std::stringstream message;
    message << "Write failed (address: " << req.address << "): " << err_msg;
    res.message = message.str();
    ROS_ERROR_STREAM_NAMED("io.write", res.message);

    return true;
  }

  ROS_DEBUG_STREAM_NAMED("io.write", "Element " << req.address << " set to: " << req.value);

  // no failure, so no need for an additional message
  res.success = true;
  return true;
}

// Service to write Group IO
bool MotomanIORelay::writeGroupIoCB(
  motoman_msgs::WriteGroupIO::Request &req,
  motoman_msgs::WriteGroupIO::Response &res)
{
  std::string err_msg;

  // send message and release mutex as soon as possible
  this->mutex_.lock();
  bool result = io_ctrl_.writeGroupIO(req.address, req.value, err_msg);
  this->mutex_.unlock();

  if (!result)
  {
    res.success = false;

    // provide caller with failure indication
    // TODO( ): should we also return the result code?
    std::stringstream message;
    message << "Group write failed (address: " << req.address << "): " << err_msg;
    res.message = message.str();
    ROS_ERROR_STREAM_NAMED("io.write", res.message);

    return true;
  }

  ROS_DEBUG_STREAM_NAMED("io.write", "Element " << req.address << " set to: " << req.value);

  // no failure, so no need for an additional message
  res.success = true;
  return true;
}

}  // namespace io_relay
}  // namespace motoman

