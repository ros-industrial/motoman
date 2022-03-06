/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, 2021, Delft Robotics Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Delft Robotics Institute, nor the names
 *    of its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 *
 * \author G.A. vd. Hoorn (TU Delft Robotics Institute)
 */

#ifndef MOTOMAN_DRIVER_IO_CTRL_H
#define MOTOMAN_DRIVER_IO_CTRL_H

#include <string>
#include "simple_message/smpl_msg_connection.h"
#include "motoman_driver/simple_message/motoman_read_mregister.h"
#include "motoman_driver/simple_message/motoman_read_mregister_reply.h"
#include "motoman_driver/simple_message/motoman_read_single_io.h"
#include "motoman_driver/simple_message/motoman_read_single_io_reply.h"
#include "motoman_driver/simple_message/motoman_read_group_io.h"
#include "motoman_driver/simple_message/motoman_read_group_io_reply.h"
#include "motoman_driver/simple_message/motoman_write_mregister.h"
#include "motoman_driver/simple_message/motoman_write_mregister_reply.h"
#include "motoman_driver/simple_message/motoman_write_single_io.h"
#include "motoman_driver/simple_message/motoman_write_single_io_reply.h"
#include "motoman_driver/simple_message/motoman_write_group_io.h"
#include "motoman_driver/simple_message/motoman_write_group_io_reply.h"

namespace motoman
{
namespace io_ctrl
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using motoman::simple_message::io_ctrl_reply::ReadMRegisterReply;
using motoman::simple_message::io_ctrl_reply::ReadSingleIOReply;
using motoman::simple_message::io_ctrl_reply::ReadGroupIOReply;
using motoman::simple_message::io_ctrl_reply::WriteMRegisterReply;
using motoman::simple_message::io_ctrl_reply::WriteSingleIOReply;
using motoman::simple_message::io_ctrl_reply::WriteGroupIOReply;

/**
 * \brief Wrapper class around Motoman-specific io control commands
 */

class MotomanIoCtrl
{
public:
  /**
   * \brief Default constructor
   */
  MotomanIoCtrl() {}

  bool init(SmplMsgConnection* connection);

public:
  /**
   * \brief Reads a single M register on the controller.
   *
   * Note: if reading was unsuccessful, the value of value is undefined.
   *
   * \param address The address (index) of the M register
   * \param value [out] Will contain the value of the M register
   * \param err_msg [out] A descriptive error message in case of failure
   * \return True IFF reading was successful
   */
  bool readMRegister(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int &value, std::string& err_msg);

  /**
   * \brief Reads a single IO point on the controller.
   *
   * Note: if reading was unsuccessful, the value of value is undefined.
   *
   * \param address The address (index) of the IO point
   * \param value [out] Will contain the value of the IO point
   * \param err_msg [out] A descriptive error message in case of failure
   * \return True IFF reading was successful
   */
  bool readSingleIO(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int &value, std::string& err_msg);

  /**
   * \brief Reads a group IO on the controller.
   *
   * Note: if reading was unsuccessful, the value of value is undefined.
   *
   * \param address The address (index) of the group IO
   * \param value [out] Will contain the value of the group IO
   * \param err_msg [out] A descriptive error message in case of failure
   * \return True IFF reading was successful
   */
  bool readGroupIO(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int &value, std::string& err_msg);

  /**
   * \brief Writes to a single M register on the controller.
   *
   * \param address The address (index) of the M register
   * \param value The value to set the M register to
   * \param err_msg [out] A descriptive error message in case of failure
   * \return True IFF writing was successful
   */
  bool writeMRegister(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value, std::string& err_msg);

  /**
   * \brief Writes to a single IO point on the controller.
   *
   * \param address The address (index) of the IO point
   * \param value The value to set the IO element to
   * \param err_msg [out] A descriptive error message in case of failure
   * \return True IFF writing was successful
   */
  bool writeSingleIO(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value, std::string& err_msg);

  /**
   * \brief Writes to a group IO on the controller.
   *
   * \param address The address (index) of the group IO
   * \param value The value to set the group IO to
   * \param err_msg [out] A descriptive error message in case of failure
   * \return True IFF writing was successful
   */
  bool writeGroupIO(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value, std::string& err_msg);

protected:
  SmplMsgConnection* connection_;

  bool sendAndReceive(industrial::shared_types::shared_int address,
    ReadMRegisterReply &reply);
  bool sendAndReceive(industrial::shared_types::shared_int address,
    ReadSingleIOReply &reply);
  bool sendAndReceive(industrial::shared_types::shared_int address,
    ReadGroupIOReply &reply);
  bool sendAndReceive(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value,
    WriteMRegisterReply &reply);
  bool sendAndReceive(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value,
    WriteSingleIOReply &reply);
  bool sendAndReceive(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value,
    WriteGroupIOReply &reply);
};

}  // namespace io_ctrl
}  // namespace motoman

#endif  // MOTOMAN_DRIVER_IO_CTRL_H
