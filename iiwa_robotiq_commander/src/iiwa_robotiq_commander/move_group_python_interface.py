#!/usr/bin/env python

# https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
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
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

## args: move_group_name - defined by us using MoveIt config wizard. Options are
##              palm_surface, finger_grasp_centroid, palm_link

class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self, move_group_name="palm_surface"):
    super(MoveGroupPythonInterface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the robot:

    self.group_names = self.robot.get_group_names()
    if move_group_name not in self.group_names:
        rospy.ROSException("Given movegroup name {} not in list of valid groups: {}".format(move_group_name, self.group_names))

    self.group = moveit_commander.MoveGroupCommander(move_group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    self.planning_frame = self.group.get_planning_frame()
    self.eef_link = self.group.get_end_effector_link()

  def print_robot_state(self):
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print self.robot.get_current_state()
    print ""

  ## args: goal_joints - list of joint values for each of the robot's joints
  def go_to_joint_state(self, goal_joints):

    num_goal_joints = len(goal_joints)
    num_robot_joints = len(self.group.get_current_joint_values())
    if num_goal_joints != num_robot_joints:
        rospy.ROSException("Gave a joint command with {} joint values, expected {} joint values.".format(num_goal_joints, num_robot_joints))

    print "now going to commanded joint state"
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.group.go(goal_joints, wait=True)
    print "at commanded joint state"
    rospy.sleep(1.)

    # Calling ``stop()`` ensures that there is no residual movement
    self.group.stop()
    rospy.sleep(1.)

    current_joints = self.group.get_current_joint_values()
    return all_close(goal_joints, current_joints, 0.01)

  ## args: pose_goal_position - [x, y, z] ee desired position
  ##       pose_goal_quat - [x, y, z, w] (geometry_msgs/Quaternion.msg order)
  ##                        quaternion representing desired ee orientation
  def go_to_pose_goal(self, pose_goal_position, pose_goal_quat):
    if len(pose_goal_position) != 3 or len(pose_goal_quat) != 4:
        rospy.ROSException("Gave a pose command with {} position values and {} quaternion values.".format(len(pose_goal_position), len(pose_goal_quat)))

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose_goal_position[0]
    pose_goal.position.y = pose_goal_position[1]
    pose_goal.position.z = pose_goal_position[2]
    pose_goal.orientation.x = pose_goal_quat[0]
    pose_goal.orientation.y = pose_goal_quat[1]
    pose_goal.orientation.z = pose_goal_quat[2]
    pose_goal.orientation.w = pose_goal_quat[3]
    self.group.set_pose_target(pose_goal)

    # TODO(mcorsaro): group.go vs. group.plan and group.execute
    # TODO(mcorsaro): add option to first display planned trajectory and wait for raw input before executing
    # TODO(mcorsaro): will this move close to unreachable goals or fail somewhere?
    # TODO(mcorsaro): same TODOs for other functions

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  # TODO(mcorsaro): turn this Cartesian path tutorial into a function or remove
  '''
  def plan_cartesian_path(self, scale=1):
    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = self.group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = self.group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL
    '''

  def display_trajectory(self, plan):
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.group.execute(plan, wait=True)

def examples():
    # Note: rosrunning this file won't work - it has to be run in the iiwa namespace. See other examples that import this file
    # TODO(mcorsaro): remove these, make dedicated example
    move_group_interface = MoveGroupPythonInterface()

    goal_joints = [0]*7
    move_group_interface.go_to_joint_state(goal_joints)
    goal_joints[1] = math.pi/2
    move_group_interface.go_to_joint_state(goal_joints)

    #move_group_interface.go_to_pose_goal()

    '''
    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)
    '''

if __name__ == '__main__':
  examples()
