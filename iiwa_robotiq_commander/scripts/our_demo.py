#!/usr/bin/env python

import rospy
import iiwa_robotiq_commander.move_group_python_interface as moveit_interface
import iiwa_robotiq_commander.robotiq_commander as robotiq_commander
import math
import copy

# Hard-coded positions
home_joints = [0]*7

box_position = [-0.0315, 0.569, 0.382]
box_orientation = [0.0, 0.7071, -0.7071, 0.0]
box_joints = [-1.3526823549857416, -0.4477169804085277, 2.7504979000288685, -1.03278923721142, 0.21950469313587695, 1.6944132471455973, -0.0769196090873275]

above_workspace_joints = [0.552735702029657, 0.36733069462288165, -0.8259502038530544, -1.0146601009543101, 0.2830675839971218, 1.8351443597777048, -0.15625835371008048]

#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

import numpy as np
import pdb

all_joint_positions = []

def hardcoded_grasp(pre_pos, pos, ori):
    # Move arm to joint configuration that hovers the hand over the workspace
    move_group_interface.go_to_joint_state(above_workspace_joints)

    # Perform a hard-coded pre-grasp
    move_group_interface.go_to_pose_goal(pre_pos, ori)

    # Perform a hard-coded grasp
    move_group_interface.go_to_pose_goal(pos, ori)

    # Close gripper
    hand_interface.close()

    # Move Back from grasp to pre, above workspace, then to box, then open gripper
    move_group_interface.go_to_pose_goal(pre_pos, ori)
    move_group_interface.go_to_joint_state(above_workspace_joints)
    move_group_interface.go_to_joint_state(box_joints)
    hand_interface.open()

    # Back to workspace
    move_group_interface.go_to_joint_state(above_workspace_joints)

if __name__ == '__main__':
    rospy.init_node('move_hardcoded_user_input')

    move_group_interface = moveit_interface.MoveGroupPythonInterface("finger_grasp_centroid")
    hand_interface = robotiq_commander.RobotiqInterface()
    move_group_interface.group.set_planning_time(1.0)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/finger_grasp_centroid', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans, rot)
        keyboard_input = raw_input("Give command")
        if keyboard_input == "w":
            print("Move forward!")
            next_trans =trans
            next_trans[0] += 0.5
            next_rot = rot

            move_group_interface.go_to_pose_goal(next_trans, next_rot)

        rate.sleep()


    """
    # Interfaces to interact with the arm and hand
    move_group_interface = moveit_interface.MoveGroupPythonInterface("finger_grasp_centroid")
    hand_interface = robotiq_commander.RobotiqInterface()

    # As defined in move_group_python_interface.py, quaternions should be in xyzw order
    # straight up
    pos = [0.31358, 0, 0.939]
    #ori = [0.5, 0.5, 0.5, 0.5]
    ori = [0,0,0,1]
    home_joints = [0]*7
    move_group_interface.group.set_planning_time(1.0)
    hand_interface = robotiq_commander.RobotiqInterface()
    move_group_interface.go_to_joint_state(home_joints)
    print("moving to goal now!")
    move_group_interface.go_to_pose_goal(pos, ori)
    """
