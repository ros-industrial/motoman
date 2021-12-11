/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Southwest Research Institute
 * Copyright (c) 2021, Delft Robotics Institute
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

#ifndef MOTOMAN_DRIVER_IO_RELAY_H
#define MOTOMAN_DRIVER_IO_RELAY_H

#include "simple_message/socket/tcp_client.h"
#include "motoman_driver/io_ctrl.h"
#include "motoman_msgs/ReadMRegister.h"
#include "motoman_msgs/ReadSingleIO.h"
#include "motoman_msgs/ReadGroupIO.h"
#include "motoman_msgs/WriteMRegister.h"
#include "motoman_msgs/WriteSingleIO.h"
#include "motoman_msgs/WriteGroupIO.h"
#include <boost/thread.hpp>

namespace motoman
{
namespace io_relay
{

using industrial::tcp_client::TcpClient;

/**
 * \brief Message handler that sends I/O service requests to the robot controller and receives the responses.
 */
class MotomanIORelay
{
public:
  /**
   * \brief Class initializer
   *
   * \param default_port
   * \return true on success, false otherwise
   */
  bool init(int default_port);

protected:
  io_ctrl::MotomanIoCtrl io_ctrl_;

  ros::ServiceServer srv_read_mregister;    // handle for read_mregister service
  ros::ServiceServer srv_read_single_io;    // handle for read_single_io service
  ros::ServiceServer srv_read_group_io;     // handle for read_group_io service
  ros::ServiceServer srv_write_mregister;   // handle for write_mregister service
  ros::ServiceServer srv_write_single_io;   // handle for write_single_io service
  ros::ServiceServer srv_write_group_io;    // handle for write_group_io service

  ros::NodeHandle node_;
  boost::mutex mutex_;
  TcpClient default_tcp_connection_;

  bool readMRegisterCB(motoman_msgs::ReadMRegister::Request &req,
                            motoman_msgs::ReadMRegister::Response &res);
  bool readSingleIoCB(motoman_msgs::ReadSingleIO::Request &req,
                            motoman_msgs::ReadSingleIO::Response &res);
  bool readGroupIoCB(motoman_msgs::ReadGroupIO::Request &req,
                            motoman_msgs::ReadGroupIO::Response &res);
  bool writeMRegisterCB(motoman_msgs::WriteMRegister::Request &req,
                            motoman_msgs::WriteMRegister::Response &res);
  bool writeSingleIoCB(motoman_msgs::WriteSingleIO::Request &req,
                            motoman_msgs::WriteSingleIO::Response &res);
  bool writeGroupIoCB(motoman_msgs::WriteGroupIO::Request &req,
                            motoman_msgs::WriteGroupIO::Response &res);
};

}  // namespace io_relay
}  // namespace motoman

#endif  // MOTOMAN_DRIVER_IO_RELAY_H
