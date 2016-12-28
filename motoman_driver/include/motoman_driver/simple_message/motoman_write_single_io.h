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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_WRITE_SINGLE_IO_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_WRITE_SINGLE_IO_H

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
namespace io_ctrl
{

/**
 * \brief Class encapsulated write single io data. Motoman specific interface
 * to write a single IO element on the controller.
 *
 * The byte representation of a write single IO command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   address             (industrial::shared_types::shared_int)    4  bytes
 *   value               (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class WriteSingleIO : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  WriteSingleIO(void);
  /**
   * \brief Destructor
   *
   */
  ~WriteSingleIO(void);

  /**
   * \brief Initializes a empty write single io command
   *
   */
  void init();

  /**
   * \brief Initializes a complete write single io command
   *
   */
  void init(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value);

  /**
   * \brief Sets address
   *
   * \param address Controller address of the targeted IO element.
   */
  void setAddress(industrial::shared_types::shared_int address)
  {
    this->address_ = address;
  }

  /**
   * \brief Sets value
   *
   * \param value Controller value of the targeted IO element.
   */
  void setValue(industrial::shared_types::shared_int value)
  {
    this->value_ = value;
  }

  /**
   * \brief Returns the address of the IO element
   *
   * \return address
   */
  industrial::shared_types::shared_int getAddress()
  {
    return this->address_;
  }

  /**
   * \brief Returns the value of the IO element
   *
   * \return value
   */
  industrial::shared_types::shared_int getValue()
  {
    return this->value_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(WriteSingleIO &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(WriteSingleIO &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 2 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief Address of IO element.
   */
  industrial::shared_types::shared_int address_;

  /**
   * \brief Value of IO element.
   */
  industrial::shared_types::shared_int value_;

};
}  // namespace io_ctrl
}  // namespace simple_message
}  // namespace motoman

#endif /* MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_WRITE_SINGLE_IO_H */
