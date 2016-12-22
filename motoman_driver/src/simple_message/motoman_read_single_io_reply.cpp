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

#include <string>
#ifdef ROS
#include "motoman_driver/simple_message/motoman_read_single_io.h"
#include "motoman_driver/simple_message/motoman_read_single_io_reply.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_read_single_io.h"
#include "motoman_read_single_io_reply.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using industrial::shared_types::shared_int;
namespace ReadSingleIOReplyResults = motoman::simple_message::io_ctrl_reply::ReadSingleIOReplyResults;

namespace motoman
{
namespace simple_message
{
namespace io_ctrl_reply
{

ReadSingleIOReply::ReadSingleIOReply(void)
{
  this->init();
}
ReadSingleIOReply::~ReadSingleIOReply(void)
{
}

void ReadSingleIOReply::init()
{
  // TODO: is '0' a good initial value?
  this->init(0/*value*/, ReadSingleIOReplyResults::SUCCESS);
}

void ReadSingleIOReply::init(shared_int value, ReadSingleIOReplyResult result_code)
{
  this->setValue(value);
  this->setResultCode(result_code);
}

std::string ReadSingleIOReply::getResultString(shared_int result_code)
{
  switch (result_code)
  {
  case ReadSingleIOReplyResults::FAILURE:
    return "Failed";
  case ReadSingleIOReplyResults::SUCCESS:
    return "Success";
  default:
    return "Unknown";
  }
}

void ReadSingleIOReply::copyFrom(ReadSingleIOReply &src)
{
  this->setValue(src.getValue());
  this->setResultCode(src.getResultCode());
}

bool ReadSingleIOReply::operator==(ReadSingleIOReply &rhs)
{
  bool rslt = this->value_ == rhs.value_ &&
              this->result_code_ == rhs.result_code_;

  return rslt;
}

bool ReadSingleIOReply::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ReadSingleIOReply load");

  if (!buffer->load(this->value_))
  {
    LOG_ERROR("Failed to load ReadSingleIOReply value");
    return false;
  }

  if (!buffer->load(this->result_code_))
  {
    LOG_ERROR("Failed to load ReadSingleIOReply result_code");
    return false;
  }

  LOG_COMM("ReadSingleIOReply data successfully loaded");
  return true;
}

bool ReadSingleIOReply::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ReadSingleIOReply unload");

  if (!buffer->unload(this->result_code_))
  {
    LOG_ERROR("Failed to unload ReadSingleIOReply result_code");
    return false;
  }

  if (!buffer->unload(this->value_))
  {
    LOG_ERROR("Failed to unload ReadSingleIOReply value");
    return false;
  }

  LOG_COMM("ReadSingleIOReply data successfully unloaded");
  return true;
}

}  // namespace io_ctrl_reply
}  // namespace simple_message
}  // namespace motoman
