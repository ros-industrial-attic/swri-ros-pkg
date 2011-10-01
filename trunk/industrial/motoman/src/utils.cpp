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

char* utils::arrayIntToChar(int* message, int sizeof_data)
// Converts int (4-byte element) array to char (1-byte element) array
// Receives and returns array pointers
// Might not work with 64-bit system
{
  char *raw_message = new char[sizeof_data];
  int int_array_length = sizeof_data/4;

  memset(raw_message, 0, sizeof_data);
  for (int i = 0; i < int_array_length; i++)
  {
    int j = 4*i;
    int array_element = message[i];
    raw_message[j+3] = ((array_element >> 24) & 0xFF);
    raw_message[j+2] = ((array_element >> 16) & 0xFF);
    raw_message[j+1] = ((array_element >> 8) & 0xFF);
    raw_message[j] = (array_element & 0xFF);
  }
  return raw_message;
}

int* utils::arrayCharToInt(char* raw_message, int sizeof_data)
// Converts char (1-byte element) array to int (4-byte element in 32-bit system) array
// Receives and returns array pointers
{
  int int_array_length = sizeof_data/4;
  int *message = new int[int_array_length];

  memset(message, 0, sizeof_data);
  memcpy(message, raw_message, sizeof_data);
  return message;
}

void utils::arrayIntToChar(int message[], char raw_message[], int sizeof_data)
// Converts int (4-byte element) array to char (1-byte element) array
// Reads and writes to array pointers
// Might not work with 64-bit system
{
  memset(raw_message, 0, sizeof_data);
  int int_array_length = sizeof_data / 4;
  for (int i = 0; i < int_array_length; i++)
  {
    int j = 4*i;
    int array_element = message[i];
    raw_message[j+3] = ((array_element >> 24) & 0xFF);
    raw_message[j+2] = ((array_element >> 16) & 0xFF);
    raw_message[j+1] = ((array_element >> 8) & 0xFF);
    raw_message[j] = (array_element & 0xFF);
  }
}

void utils::arrayCharToInt(char raw_message[], int message[], int sizeof_data)
// Converts char (1-byte element) array to int (4-byte element in 32-bit Linux) array
// Reads and writes to array pointers
{
  memset(message, 0, sizeof_data);
  memcpy(message, raw_message, sizeof_data);
}
