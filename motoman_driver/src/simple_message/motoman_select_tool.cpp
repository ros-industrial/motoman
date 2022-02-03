/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, Delft Robotics Institute
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
#include "motoman_driver/simple_message/motoman_select_tool.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_select_tool.h"     // NOLINT(build/include)
#include "shared_types.h"            // NOLINT(build/include)
#include "log_wrapper.h"             // NOLINT(build/include)
#endif

using industrial::shared_types::shared_int;

namespace motoman
{
namespace simple_message
{
namespace misc
{
SelectTool::SelectTool(void)
{
  this->init();
}

SelectTool::~SelectTool(void)
{
}

void SelectTool::init()
{
  // TODO( ): are 0s good initial values?
  this->init(0, 0);
}

void SelectTool::init(shared_int group_number,
    shared_int tool_number,
    shared_int sequence_number)
{
  this->setGroupNumber(group_number);
  this->setToolNumber(tool_number);
  this->setSequenceNumber(sequence_number);
}

void SelectTool::copyFrom(SelectTool &src)
{
  this->setGroupNumber(src.getGroupNumber());
  this->setToolNumber(src.getToolNumber());
  this->setSequenceNumber(src.getSequenceNumber());
}

bool SelectTool::operator==(const SelectTool &rhs) const
{
  bool rslt = this->group_number_ == rhs.group_number_
    && this->tool_number_ == rhs.tool_number_
    && this->sequence_number_ == rhs.sequence_number_;

  return rslt;
}

bool SelectTool::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SelectTool command load");

  if (!buffer->load(this->group_number_))
  {
    LOG_ERROR("Failed to load SelectTool group number");
    return false;
  }

  if (!buffer->load(this->tool_number_))
  {
    LOG_ERROR("Failed to load SelectTool tool number");
    return false;
  }

  if (!buffer->load(this->sequence_number_))
  {
    LOG_ERROR("Failed to load SelectTool sequence number");
    return false;
  }

  LOG_COMM("SelectTool data successfully loaded");
  return true;
}

bool SelectTool::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SelectTool command unload");

  if (!buffer->unload(this->sequence_number_))
  {
    LOG_ERROR("Failed to unload SelectTool sequence number");
    return false;
  }

  if (!buffer->unload(this->tool_number_))
  {
    LOG_ERROR("Failed to unload SelectTool tool number");
    return false;
  }

  if (!buffer->unload(this->group_number_))
  {
    LOG_ERROR("Failed to unload SelectTool group number");
    return false;
  }

  LOG_COMM("SelectTool data successfully unloaded");
  return true;
}

}  // namespace misc
}  // namespace simple_message
}  // namespace motoman
