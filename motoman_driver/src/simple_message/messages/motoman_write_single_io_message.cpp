/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Delft Robotics Institute
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

#ifdef ROS
#include "motoman_driver/simple_message/messages/motoman_write_single_io_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_write_single_io_message.h"  // NOLINT(build/include)
#include "byte_array.h"                       // NOLINT(build/include)
#include "log_wrapper.h"                      // NOLINT(build/include)
#endif

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using motoman::simple_message::io_ctrl::WriteSingleIO;

namespace motoman
{
namespace simple_message
{
namespace io_ctrl_message
{
WriteSingleIOMessage::WriteSingleIOMessage(void)
{
  this->init();
}

WriteSingleIOMessage::~WriteSingleIOMessage(void)
{
}

bool WriteSingleIOMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  if (!data.unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload WriteSingleIO data");
    return false;
  }

  return true;
}

void WriteSingleIOMessage::init(WriteSingleIO & cmd)
{
  this->init();
  this->cmd_.copyFrom(cmd);
}

void WriteSingleIOMessage::init()
{
  this->setMessageType(MotomanMsgTypes::MOTOMAN_WRITE_SINGLE_IO);
  this->cmd_.init();
}

bool WriteSingleIOMessage::load(ByteArray *buffer)
{
  LOG_COMM("Executing WriteSingleIO message load");
  if (!buffer->load(this->cmd_))
  {
    LOG_ERROR("Failed to load WriteSingleIO message");
    return false;
  }

  return true;
}

bool WriteSingleIOMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing WriteSingleIO message unload");

  if (!buffer->unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload WriteSingleIO message");
    return false;
  }

  return true;
}

}  // namespace io_ctrl_message
}  // namespace simple_message
}  // namespace motoman

