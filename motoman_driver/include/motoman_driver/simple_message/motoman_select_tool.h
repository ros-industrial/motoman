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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SELECT_TOOL_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SELECT_TOOL_H

#ifdef ROS
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "simple_serialize.h"  // NOLINT(build/include)
#include "shared_types.h"      // NOLINT(build/include)
#include "log_wrapper.h"       // NOLINT(build/include)
#endif

namespace motoman
{
namespace simple_message
{
namespace misc
{

/**
 * \brief Class encapsulated select tool data. Motoman specific interface
 * to select a specific tool file on the controller.
 *
 * The byte representation of a select tool command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   group_number        (industrial::shared_types::shared_int)    4  bytes
 *   tool_number         (industrial::shared_types::shared_int)    4  bytes
 *   sequence_number     (industrial::shared_types::shared_int)    4  bytes
 *
 * Note: sequence_number is marked as optional by MotoROS. It will be included
 * in the SimpleMessage service request, but does not have to be set to any
 * specific value. It can be used as a "tracking number" that will be "echoed
 * back in the response message" (according to MotoROS source code comments).
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class SelectTool : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SelectTool(void);

  /**
   * \brief Destructor
   *
   */
  ~SelectTool(void);

  /**
   * \brief Initializes a empty select tool command
   *
   */
  void init();

  /**
   * \brief Initializes a complete select tool command
   *
   * Note: sequence_number default value of 0.
   */
  void init(industrial::shared_types::shared_int group_number,
    industrial::shared_types::shared_int tool_number,
    industrial::shared_types::shared_int sequence_number = 0);

  /**
   * \brief Sets group number
   *
   * \param group_number Controller group the tool file is defined for
   */
  void setGroupNumber(industrial::shared_types::shared_int group_number)
  {
    this->group_number_ = group_number;
  }

  /**
   * \brief Returns the group number
   *
   * \return group number
   */
  industrial::shared_types::shared_int getGroupNumber()
  {
    return this->group_number_;
  }

  /**
   * \brief Sets tool number
   *
   * \param tool_number Tool file identifier
   */
  void setToolNumber(industrial::shared_types::shared_int tool_number)
  {
    this->tool_number_ = tool_number;
  }

  /**
   * \brief Returns the tool file identifier
   *
   * \return tool number
   */
  industrial::shared_types::shared_int getToolNumber()
  {
    return this->tool_number_;
  }

  /**
   * \brief Sets sequence number
   *
   * \param sequence_number Optional sequence number
   */
  void setSequenceNumber(industrial::shared_types::shared_int sequence_number)
  {
    this->sequence_number_ = sequence_number;
  }

  /**
   * \brief Returns the optional sequence number
   *
   * \return sequence number
   */
  industrial::shared_types::shared_int getSequenceNumber()
  {
    return this->sequence_number_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(SelectTool &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(const SelectTool &rhs) const;

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer) override;
  bool unload(industrial::byte_array::ByteArray *buffer) override;
  unsigned int byteLength() override
  {
    return 3 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief The group number for which the tool file is defined.
   */
  industrial::shared_types::shared_int group_number_;

  /**
   * \brief The tool file number.
   */
  industrial::shared_types::shared_int tool_number_;

  /**
   * \brief Optional sequence number (for tracking tool select invocations).
   */
  industrial::shared_types::shared_int sequence_number_;
};
}  // namespace misc
}  // namespace simple_message
}  // namespace motoman

#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SELECT_TOOL_H
