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
#include "motoman_driver/simple_message/motoman_write_single_io.h"
#include "motoman_driver/simple_message/motoman_write_single_io_reply.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_write_single_io.h"
#include "motoman_write_single_io_reply.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using industrial::shared_types::shared_int;
namespace WriteSingleIOReplyResults = motoman::simple_message::io_ctrl_reply::WriteSingleIOReplyResults;

namespace motoman
{
namespace simple_message
{
namespace io_ctrl_reply
{

WriteSingleIOReply::WriteSingleIOReply(void)
{
  this->init();
}
WriteSingleIOReply::~WriteSingleIOReply(void)
{
}

void WriteSingleIOReply::init()
{
  // TODO: is success a good initial value?
  this->init(WriteSingleIOReplyResults::SUCCESS);
}

void WriteSingleIOReply::init(WriteSingleIOReplyResult result_code)
{
  this->setResultCode(result_code);
}

std::string WriteSingleIOReply::getResultString(shared_int result_code)
{
  switch (result_code)
  {
  case WriteSingleIOReplyResults::FAILURE:
    return "Failed";
  case WriteSingleIOReplyResults::SUCCESS:
    return "Success";
  default:
    return "Unknown";
  }
}

void WriteSingleIOReply::copyFrom(WriteSingleIOReply &src)
{
  this->setResultCode(src.getResultCode());
}

bool WriteSingleIOReply::operator==(WriteSingleIOReply &rhs)
{
  bool rslt = this->result_code_ == rhs.result_code_;

  return rslt;
}

bool WriteSingleIOReply::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing WriteSingleIOReply load");

  if (!buffer->load(this->result_code_))
  {
    LOG_ERROR("Failed to load WriteSingleIOReply result_code");
    return false;
  }

  LOG_COMM("WriteSingleIOReply data successfully loaded");
  return true;
}

bool WriteSingleIOReply::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing WriteSingleIOReply unload");

  if (!buffer->unload(this->result_code_))
  {
    LOG_ERROR("Failed to unload WriteSingleIOReply result_code");
    return false;
  }

  LOG_COMM("WriteSingleIOReply data successfully unloaded");
  return true;
}

}  // namespace io_ctrl_reply
}  // namespace simple_message
}  // namespace motoman
