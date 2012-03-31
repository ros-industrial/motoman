/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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
 */

#ifndef SIMPLE_MESSAGE_CONNECTION_H
#define SIMPLE_MESSAGE_CONNECTION_H

#ifdef ROS

#include "simple_message/byte_array.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"

#endif

#ifdef MOTOPLUS

#include "byte_array.h"
#include "simple_message.h"
#include "shared_types.h"

#endif


namespace industrial
{
namespace smpl_msg_connection
{

class SmplMsgConnection

{
public:

  // Message
  virtual bool sendMsg(industrial::simple_message::SimpleMessage & message);
  virtual bool receiveMsg(industrial::simple_message::SimpleMessage & message);
  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage & send,
                         industrial::simple_message::SimpleMessage & recv, 
                         bool verbose = false);

  /**
   * \brief return connection status
   *
   * \return true if connected
   */
  virtual bool isConnected()=0;

  /**
   * \brief connects to the remote host
   *
   * \return true on success, false otherwise
   */
  virtual bool makeConnect()=0;

private:

  // Overrides
  virtual bool sendBytes(industrial::byte_array::ByteArray & buffer) =0;
  virtual bool receiveBytes(industrial::byte_array::ByteArray & buffer,
                            industrial::shared_types::shared_int num_bytes) =0;

};

} //namespace message_connection
} //namespace industrial

#endif //SIMPLE_MESSAGE_CONNECTION_H
