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

#include "byte_array.h"
#include "simple_message.h"

namespace industrial
{
namespace smpl_msg_connection
{

class SmplMsgConnection

{
public:

  SmplMsgConnection();
  ~SmplMsgConnection();


  // Overrides
  virtual bool send(industrial::byte_array::ByteArray & buffer);
  virtual bool receive(industrial::byte_array::ByteArray & buffer, size_t num_bytes);

  // Message
  bool send(industrial::simple_message::SimpleMessage & message);

  // Do not override receive, it has logic to automatically respond to pings.  Enabling
  // this at a low level is important.  It ensures ping logic is robust and not subject
  // to programmer induced bugs.
  bool receive(industrial::simple_message::SimpleMessage & message);
  bool sendAndReceive(industrial::simple_message::SimpleMessage & send,
                      industrial::simple_message::SimpleMessage & recv);

private:


  // If you must override receive, then override this one.
  bool receiveAllMsgs(industrial::simple_message::SimpleMessage & message);

};

} //namespace message_connection
} //namespace industrial

#endif //SIMPLE_MESSAGE_CONNECTION_H

