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


#include "stdio.h"  //printf

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


// By default we will log to printf.  Other types of logging (if defined) will
// override these definitions below.
#define SIMP_LOGGER
#ifdef SIMP_LOGGER


#define LOG_DEBUG(format, ...)  \
  printf(format, ##__VA_ARGS__)

#define LOG_INFO(format, ...)  \
    printf(format, ##__VA_ARGS__)

#define LOG_WARN(format, ...)  \
    printf(format, ##__VA_ARGS__)

#define LOG_ERROR(format, ...)  \
    printf(format, ##__VA_ARGS__)

#define LOG_FATAL(format, ...)  \
    printf(FATAL, ##__VA_ARGS__)

#endif //SIMP_LOGGER


// Define ROS_LOGGER if this library will execute under ROS
#ifdef ROS_LOGGER

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

#endif //ROS_LOGGER



// Define MOTOPLUS_LOGGER if this library will execute under MOTOPLUS
#ifdef MOTOPLUS_LOGGER

#define LOG_DEBUG(format, ...)  \
  printf(format, ##__VA_ARGS__)

#define LOG_INFO(format, ...)  \
    printf(format, ##__VA_ARGS__)

#define LOG_WARN(format, ...)  \
    printf(format, ##__VA_ARGS__)

#define LOG_ERROR(format, ...)  \
    printf(format, ##__VA_ARGS__)

#define LOG_FATAL(format, ...)  \
    printf(FATAL, ##__VA_ARGS__)

#endif //MOTPLUS_LOGGER

} // namespace industrial
} // namespace loge_wrapper

#endif /* LOG_WRAPPER_H_ */
