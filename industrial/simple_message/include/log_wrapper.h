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

#ifndef LOG_WRAPPER_H_
#define LOG_WRAPPER_H_

#ifdef ROS
#include "ros/ros.h"
#endif

namespace industrial
{

/**
 * \brief Contains macro that wrap standard logging calls.  Wrapping logging
 * calls allows libraries to be used inside and outside the ROS framework.
 *
 * Macros are used because passing variable argument lists are much easier
 * than passing them through functions.
 */
namespace log_wrapper
{
    

// Define ROS if this library will execute under ROS
#ifdef ROS

#define LOG_DEBUG(format, ...)  \
  ROS_DEBUG(format, ##__VA_ARGS__)

#define LOG_INFO(format, ...)  \
  ROS_INFO(format, ##__VA_ARGS__)

#define LOG_WARN(format, ...)  \
  ROS_WARN(format, ##__VA_ARGS__)

#define LOG_ERROR(format, ...)  \
  ROS_ERROR(format, ##__VA_ARGS__)

#define LOG_FATAL(format, ...)  \
  ROS_DEBUG(FATAL, ##__VA_ARGS__)

#endif //ROS



// Define MOTOPLUS if this library will execute under MOTOPLUS
#ifdef MOTOPLUS

#define LOG_DEBUG(format, ...)  do { \
  printf("DEBUG: "); \
  printf(format, ##__VA_ARGS__); \
  printf("\n") \
  while(0)

#define LOG_INFO(format, ...)  do {  \
    printf("INFO: "); \
    printf(format, ##__VA_ARGS__); \
    printf("\n") \
    while(0)

#define LOG_WARN(format, ...)  do {  \
    printf("WARN: "); \
    printf(format, ##__VA_ARGS__); \
    printf("\n") \
    while(0)

#define LOG_ERROR(format, ...)  do {  \
    printf("ERROR: "); \
    printf(format, ##__VA_ARGS__); \
    printf("\n") \
    while(0)

#define LOG_FATAL(format, ...)  do {  \
    printf("FATAL: "); \
    printf(FATAL, ##__VA_ARGS__); \
    printf("\n") \
    while(0)

#endif //MOTPLUS



} // namespace industrial
} // namespace loge_wrapper

#endif /* LOG_WRAPPER_H_ */
