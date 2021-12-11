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
#include "motoman_driver/simple_message/motoman_read_group_io.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_read_group_io.h"   // NOLINT(build/include)
#include "shared_types.h"            // NOLINT(build/include)
#include "log_wrapper.h"             // NOLINT(build/include)
#endif

using industrial::shared_types::shared_int;

namespace motoman
{
namespace simple_message
{
namespace io_ctrl
{
ReadGroupIO::ReadGroupIO(void)
{
  this->init();
}
ReadGroupIO::~ReadGroupIO(void)
{
}

void ReadGroupIO::init()
{
  // TODO( ): is '0' a good initial value?
  this->init(0);
}

void ReadGroupIO::init(shared_int address)
{
  this->setAddress(address);
}

void ReadGroupIO::copyFrom(ReadGroupIO &src)
{
  this->setAddress(src.getAddress());
}

bool ReadGroupIO::operator==(ReadGroupIO &rhs)
{
  bool rslt = this->address_ == rhs.address_;

  return rslt;
}

bool ReadGroupIO::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ReadGroupIO command load");

  if (!buffer->load(this->address_))
  {
    LOG_ERROR("Failed to load ReadGroupIO address");
    return false;
  }

  LOG_COMM("ReadGroupIO data successfully loaded");
  return true;
}

bool ReadGroupIO::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ReadGroupIO command unload");

  if (!buffer->unload(this->address_))
  {
    LOG_ERROR("Failed to unload ReadGroupIO address");
    return false;
  }

  LOG_COMM("ReadGroupIO data successfully unloaded");
  return true;
}

}  // namespace io_ctrl
}  // namespace simple_message
}  // namespace motoman
