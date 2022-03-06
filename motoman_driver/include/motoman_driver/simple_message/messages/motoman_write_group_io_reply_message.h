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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MESSAGES_MOTOMAN_WRITE_GROUP_IO_REPLY_MESSAGE_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MESSAGES_MOTOMAN_WRITE_GROUP_IO_REPLY_MESSAGE_H

#ifdef ROS
#include "simple_message/typed_message.h"
#include "simple_message/shared_types.h"
#include "motoman_driver/simple_message/motoman_simple_message.h"
#include "motoman_driver/simple_message/motoman_write_group_io_reply.h"

#endif

#ifdef MOTOPLUS
#include "typed_message.h"                  // NOLINT(build/include)
#include "shared_types.h"                   // NOLINT(build/include)
#include "motoman_simple_message.h"         // NOLINT(build/include)
#include "motoman_write_group_io_reply.h"   // NOLINT(build/include)

#endif

namespace motoman
{
namespace simple_message
{
namespace io_ctrl_reply_message
{


/**
 * \brief Class encapsulated motoman write group io reply message generation
 * methods (either to or from a industrial::simple_message::SimpleMessage type).
 *
 * This message simply wraps the following data type:
 *   motoman::simple_message::io_ctrl_reply::WriteGroupIOReply
 * The data portion of this typed message matches WriteGroupIOReply exactly.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class WriteGroupIOReplyMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  WriteGroupIOReplyMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~WriteGroupIOReplyMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a write group io reply structure
   *
   * \param reply write group io reply data structure
   *
   */
  void init(motoman::simple_message::io_ctrl_reply::WriteGroupIOReply & reply);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->reply_.byteLength();
  }

  motoman::simple_message::io_ctrl_reply::WriteGroupIOReply reply_;

private:
};
}  // namespace io_ctrl_reply_message
}  // namespace simple_message
}  // namespace motoman

#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_MESSAGES_MOTOMAN_WRITE_GROUP_IO_REPLY_MESSAGE_H
