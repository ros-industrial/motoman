﻿/*
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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_READ_SINGLE_IO_REPLY_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_READ_SINGLE_IO_REPLY_H

#include <string>
#ifdef ROS
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "simple_serialize.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

namespace motoman
{
namespace simple_message
{
namespace io_ctrl_reply
{

/**
 * \brief Enumeration of Read Single IO reply result codes.
 */
namespace ReadSingleIOReplyResults
{
enum ReadSingleIOReplyResult
{
  FAILURE    = 0,
  SUCCESS    = 1
};
}
typedef ReadSingleIOReplyResults::ReadSingleIOReplyResult ReadSingleIOReplyResult;

/**
 * \brief Class encapsulated read single io reply data.  These messages are sent
 * by the FS100 controller in response to ReadSingleIO messages.
 *
 * The byte representation of a read single io reply is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   value               (industrial::shared_types::shared_int)    4  bytes
 *   result_code         (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class ReadSingleIOReply : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  ReadSingleIOReply(void);
  /**
   * \brief Destructor
   *
   */
  ~ReadSingleIOReply(void);

  /**
   * \brief Initializes a empty motion control reply
   *
   */
  void init();

  /**
   * \brief Initializes a complete read single io reply
   *
   */
  void init(industrial::shared_types::shared_int value,
            ReadSingleIOReplyResult result_code);

  /**
   * \brief Sets value
   *
   * \param value The value
   */
  void setValue(industrial::shared_types::shared_int value)
  {
    this->value_ = value;
  }

  /**
   * \brief Returns the value
   *
   * \return value
   */
  industrial::shared_types::shared_int getValue() const
  {
    return this->value_;
  }

  /**
   * \brief Sets the result code
   *
   * \param result code
   */
  void setResultCode(industrial::shared_types::shared_int result_code)
  {
    this->result_code_ = result_code;
  }

  /**
   * \brief Returns the result code
   *
   * \return result_code number
   */
  industrial::shared_types::shared_int getResultCode() const
  {
    return this->result_code_;
  }

  /*
   * \brief Returns a string interpretation of a result code
   * \param code result code
   * \return string message associated with result code
   */
  static std::string getResultString(industrial::shared_types::shared_int code);
  std::string getResultString() const
  {
    return getResultString(this->result_code_);
  }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(ReadSingleIOReply &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(ReadSingleIOReply &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 2 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief The value
   */
  industrial::shared_types::shared_int value_;

  /**
   * \brief The result code
   */
  industrial::shared_types::shared_int result_code_;
};
}  // namespace io_ctrl_reply
}  // namespace simple_message
}  // namespace motoman

#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_READ_SINGLE_IO_REPLY_H
