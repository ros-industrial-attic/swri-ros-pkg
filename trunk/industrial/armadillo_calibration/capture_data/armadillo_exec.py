#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('armadillo_calibration')
import rospy
import yaml
import sys
import capture_executive.capture_exec
import time
from calibration_msgs.msg import RobotMeasurement
import os
import string

print "Starting executive..."
time.sleep(2.0)

rospy.init_node("armadillo_capture_executive_node")

samples_dir = rospy.myargv()[1]
config_dir = rospy.myargv()[2]

executive = capture_executive.capture_exec.CaptureExecutive(config_dir)
time.sleep(1.0)

sample_names = [x for x in os.listdir(samples_dir) if ".yaml" in x]
sample_names.sort()

print "Samples: \n - %s" % "\n - ".join(sample_names)

pub = rospy.Publisher("robot_measurement", RobotMeasurement)

success_count = 0
fail_count = 0

try:
    # Capture Data
    full_paths = [samples_dir + x for x in sample_names]

    cur_config = yaml.load(open(full_paths[0]))
    m_robot = executive.capture(cur_config, rospy.Duration(0.01))

    print "Please put the checkerboard in the hand (open/close the gripper with the joystick's square/circle buttons). press <enter> to continue..."
    resp = raw_input("press <enter> ")
    if string.upper(resp) == "N":
        print "Skipping samples"
    else:
        for cur_sample_path in full_paths:
            print "On sample [%s]" % cur_sample_path
            cur_config = yaml.load(open(cur_sample_path))
            m_robot = executive.capture(cur_config, rospy.Duration(20))
            if m_robot is None:
                print "--------------- Failed To Capture a Sample -----------------"
                fail_count += 1
            else:
                print "++++++++++++++ Successfully Captured a Sample ++++++++++++++"
                success_count += 1
                pub.publish(m_robot)
            print "Succeded on %u/%u samples" % (success_count, fail_count + success_count)
            if rospy.is_shutdown():
                break

except EOFError:
    print "Exiting"

time.sleep(1)

print "Calibration data collection has completed!"
print ""
print "You can now kill this node, along with any other calibration nodes that are running."

