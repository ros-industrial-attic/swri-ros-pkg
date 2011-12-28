/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#ifndef TYPED_MESSAGE_H
#define TYPED_MESSAGE_H

#include "simple_message.h"

namespace industrial
{
namespace typed_message
{

/**
 * \brief Message interface for typed messages built from simple_message(link)
 */
//* TypedMessage
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class TypedMessage
{

  /*
   * This class is just put here for planning purposes.  After many typed messages
   * have been created the common code will be ported to this class.
   */

public:
  /**
   * \brief Initializes message from a simple message
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool init(industrial::simple_message::SimpleMessage & msg)=0;

  /**
   * \brief Initializes a new empty message
   *
   */
  virtual void init()=0;

  /**
   * \brief creates a simple_message request
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toRequest(industrial::simple_message::SimpleMessage & msg)=0;

  /**
   * \brief creates a simple_message reply
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toReply(industrial::simple_message::SimpleMessage & msg)=0;

  /**
   * \brief creates a simple_message topic
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toTopic(industrial::simple_message::SimpleMessage & msg)=0;

  /**
     * \brief gets message type (enumeration)
     *
     * \return message type
     */
  int getMessageType() const
  {
    return message_type_;
  }

protected:


  void setMessageType(int MESSAGE_TYPE = industrial::simple_message::StandardMsgTypes::INVALID)
  {
    this->message_type_ = MESSAGE_TYPE;
  }

private:

  /**
   * \brief Message type expected by callback
   */

  int message_type_;

};

}
}

#endif /* TYPED_MESSAGE_H */
