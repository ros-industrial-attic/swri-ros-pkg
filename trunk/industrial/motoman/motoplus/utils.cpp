/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Yaskawa America, Inc.
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
*       * Neither the name of the Yaskawa America, Inc., nor the names 
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

#include "utils.h"

CHAR* utils::arrayIntToChar(LONG* message, LONG sizeof_data)
// Converts LONG (4-byte element in 32-bit VxWorks) array to CHAR (1-byte element) array
// Receives and returns array pointers
{
  CHAR *raw_message = new CHAR[sizeof_data];
  LONG LONG_array_length = sizeof_data/4;

  memset(raw_message, 0, sizeof_data);
  for (LONG i = 0; i < LONG_array_length; i++)
  {
    LONG j = 4*i;
    LONG array_element = message[i];
    raw_message[j+3] = ((array_element >> 24) & 0xFF);
    raw_message[j+2] = ((array_element >> 16) & 0xFF);
    raw_message[j+1] = ((array_element >> 8) & 0xFF);
    raw_message[j] = (array_element & 0xFF);
  }
  return raw_message;
}

LONG* utils::arrayCharToInt(CHAR* raw_message, LONG sizeof_data)
// Converts CHAR (1-byte element) array to LONG (4-byte element in 32-bit VxWorks) array
// Receives and returns array pointers
{
  LONG LONG_array_length = sizeof_data/4;
  LONG *message = new LONG[LONG_array_length];

  memset(message, 0, sizeof_data);
  memcpy(message, raw_message, sizeof_data);
  return message;
}

void utils::arrayIntToChar(LONG message[], CHAR raw_message[], LONG sizeof_data)
// Converts LONG (4-byte element) array to CHAR (1-byte element) array
// Reads and writes to array pointers
{
  memset(raw_message, 0, sizeof_data);
  LONG array_length = sizeof_data / 4;
  for (SHORT i = 0; i < array_length; i++)
  {
    LONG j = 4*i;
    LONG array_element = message[i];
    raw_message[j+3] = ((array_element >> 24) & 0xFF);
    raw_message[j+2] = ((array_element >> 16) & 0xFF);
    raw_message[j+1] = ((array_element >> 8) & 0xFF);
    raw_message[j] = (array_element & 0xFF);
  }
}

void utils::arrayCharToInt(CHAR raw_message[], LONG message[], LONG sizeof_data)
// Converts CHAR (1-byte element) array to LONG (4-byte element) array
// Reads and writes to array pointers
{
  memset(message, 0, sizeof_data);
  memcpy(message, raw_message, sizeof_data);
}