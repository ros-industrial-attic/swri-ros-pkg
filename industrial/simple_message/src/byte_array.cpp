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

#include "byte_array.h"
#include "simple_serialize.h"
#include "log_wrapper.h"

#include "string.h"


namespace industrial
{
namespace byte_array
{

using namespace industrial::simple_serialize;
using namespace industrial::shared_types;


ByteArray::ByteArray(void)
{
  this->init();
}


ByteArray::~ByteArray(void)
{
}


void ByteArray::init()
{
  memset(&(buffer_[0]), 0, this->MAX_SIZE);
  this->setBufferSize(0);
}


bool ByteArray::init(const char* buffer, const shared_int byte_size)
{
  bool rtn;

  if ( this->MAX_SIZE >= byte_size )
  {
    this->load((void*)buffer, byte_size);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to initialize byte array, buffer size: %u greater than max: %u",
              byte_size, this->getMaxBufferSize());
    rtn = false;
  }
  return rtn;
}


void ByteArray::copyFrom(ByteArray & buffer)
{
  if (buffer.getBufferSize() != 0)
  {
    this->setBufferSize(buffer.getBufferSize());
    memcpy(this->getRawDataPtr(), buffer.getRawDataPtr(), this->buffer_size_);
  }
  else
  {
    LOG_WARN("Byte array copy not performed, buffer to copy is empty");
  }
}


char* ByteArray::getRawDataPtr()
{
  if (0 != this->buffer_size_)
  {
    return &this->buffer_[0];
  }
  else
  {
    LOG_ERROR("Failed to return char buffer pointer, buffer is empty");
    return NULL;
  }
}

/****************************************************************
 // load(*)
 //
 // Methods for loading various data types.
 //
 */
bool ByteArray::load(shared_bool value)
{
  return this->load(&value, sizeof(shared_bool));
}

bool ByteArray::load(shared_real value)
{
  return this->load(&value,  sizeof(shared_real));
}

bool ByteArray::load(shared_int value)
{
  return this->load(&value, sizeof(shared_int));
}

bool ByteArray::load(SimpleSerialize &value)
{
  return value.load(this);
}

bool ByteArray::load(ByteArray &value)
{
  return this->load(value.getRawDataPtr(), value.getBufferSize());
}


bool ByteArray::load(void* value, const shared_int byte_size)
{

  bool rtn;
  // Get the load pointer before extending the buffer.
  char* loadPtr;

  // Check inputs
  if ( NULL == value)
  {
    LOG_ERROR("NULL point passed into load method");
    return false;
  }

  loadPtr = this->getLoadPtr();

  if (this->extendBufferSize(byte_size))
  {
    memcpy(loadPtr, value, byte_size);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to load byte array");
    rtn = false;
  }

  return rtn;
}

/****************************************************************
 // unload(*)
 //
 // Methods for unloading various data types.  Unloading data shortens
 // the internal buffer.  The resulting memory that holds the data is
 // lost.
 //
 */
bool ByteArray::unload(shared_bool & value)
{
  return this->unload(&value, sizeof(shared_bool));
}

bool ByteArray::unload(shared_real &value)
{
  return this->unload(&value, sizeof(shared_real));
}

bool ByteArray::unload(shared_int &value)
{
  return this->unload(&value, sizeof(shared_int));
}

bool ByteArray::unload(SimpleSerialize &value)
{
  return value.unload(this);
}

bool ByteArray::unload(ByteArray &value, const shared_int byte_size)
{
  char* unloadPtr = this->getUnloadPtr(byte_size);
  bool rtn;

  if ( NULL != unloadPtr)
  {
    rtn = value.unload(unloadPtr, byte_size);
  }
  else
  {
    LOG_ERROR("Unload pointer returned NULL");
    rtn = false;
  }

  return rtn;
}

bool ByteArray::unload(void* value, shared_int byteSize)
{
  bool rtn;
  char* unloadPtr;


  // Check inputs
  if ( NULL == value)
  {
    LOG_ERROR("NULL point passed into unload method");
    return false;
  }

  unloadPtr = this->getUnloadPtr(byteSize);

  if ( NULL != unloadPtr )
  {

    if (this->shortenBufferSize(byteSize))
    {
      memcpy(value, unloadPtr, byteSize);
      rtn = true;
    }
    else
    {
      LOG_ERROR("Failed to shorten array");
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Unload pointer returned NULL");
    rtn = false;
  }

  return rtn;
}





unsigned int ByteArray::getBufferSize()
{
  return this->buffer_size_;
}


unsigned int ByteArray::getMaxBufferSize()
{
  return this->MAX_SIZE;
}


bool ByteArray::setBufferSize(shared_int size)
{
  bool rtn;

  if (this->MAX_SIZE >= size)
  {
    this->buffer_size_ = size;
    rtn = true;
  }
  else
  {
    LOG_ERROR("Set buffer size: %u, larger than MAX:, %u",
              size, this->MAX_SIZE);
    rtn = false;
  }

  return rtn;

}

bool ByteArray::extendBufferSize(shared_int size)
{
  unsigned int newSize;

  newSize = this->getBufferSize() + size;
  return this->setBufferSize(newSize);

}

bool ByteArray::shortenBufferSize(shared_int size)
{
  unsigned int newSize;
  bool rtn;

  // If the buffer is not larger than the size it is shortened by
  // we fail.  This is checked here (as opposed to setBufferSize)
  // because setBufferSize assumes a unsigned argument and therefore
  // wouldn't catch a negative size.
  if (size <= this->getBufferSize())
  {
    newSize = this->getBufferSize() - size;
    rtn = this->setBufferSize(newSize);
  }
  else
  {
    LOG_ERROR("Failed to shorten buffer by %u bytes, buffer too small, %u bytes",
              size, this->getBufferSize());
    rtn = false;
  }

  return rtn;

}

char* ByteArray::getLoadPtr()
{

  return &this->buffer_[this->buffer_size_];
}

char* ByteArray::getUnloadPtr(shared_int byteSize)
{
  char* rtn;

  if (byteSize <= this->getBufferSize())
  {
    rtn = this->getLoadPtr() - byteSize;
  }
  else
  {
    LOG_ERROR("Get unload pointer failed, buffer too small");
    rtn = NULL;
  }

  return rtn;
}

} // namespace byte_array
} // namespace industrial
