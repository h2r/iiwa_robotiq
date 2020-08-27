#!/usr/bin/env python


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
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
#  * Neither the name of Robotiq, Inc. nor the names of its
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
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Class for sending simple commands to a ROS node controlling a 3F gripper, adapted from Command-line interface.

This serves as an example for publishing messages on the 'Robotiq3FGripperRobotOutput' topic using the 'Robotiq3FGripper_robot_output' msg type for sending commands to a 3F gripper gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

#https://github.com/ros-industrial/robotiq/blob/kinetic-devel/robotiq_3f_gripper_control/nodes/Robotiq3FGripperSimpleController.py

import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

class RobotiqInterface(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=1, latch=True)
        self.activated = False

        self.command = Robotiq3FGripperRobotOutput();

    def activate(self, delay=20):
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSPA = 255
        self.command.rFRA = 150
        self.publisher.publish(self.command)
        self.activated = True
        print "Now sleeping for {} seconds while the hand activates.".format(delay)
        rospy.sleep(delay)
        return True

    def basic_mode(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rMOD = 0
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def wide_mode(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rMOD = 2
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def pinch_mode(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rMOD = 1
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def scissor_mode(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rMOD = 3
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def close(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rPRA = 255
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def open(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rPRA = 0
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def reset(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rACT = 0
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def go_to(self, value, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        if type(value) == float:
            value = int(value)
        if type(value) != int or value > 255 or value < 0:
            print "Could not send fingers to", value, "- give int between 0 and 255"
            return False
        self.command.rPRA = value
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def faster(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rSPA += 25
        if self.command.rSPA > 255:
            self.command.rSPA = 255
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def slower(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rSPA -= 25
        if self.command.rSPA < 0:
            self.command.rSPA = 0
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def increase_force(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rFRA += 25
        if self.command.rFRA > 255:
            self.command.rFRA = 255
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True

    def decrease_force(self, delay=3):
        if not self.activated:
            print "Activate the hand before using it."
            return False
        self.command.rFRA -= 25
        if self.command.rFRA < 0:
            self.command.rFRA = 0
        self.publisher.publish(self.command)
        print "Now sleeping for {} seconds while the hand executes given command.".format(delay)
        rospy.sleep(delay)
        return True
