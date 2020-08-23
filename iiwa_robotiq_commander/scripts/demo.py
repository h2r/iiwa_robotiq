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
    
    # Interfaces to interact with the arm and hand
    move_group_interface = moveit_interface.MoveGroupPythonInterface("finger_grasp_centroid")
    hand_interface = robotiq_commander.RobotiqInterface()
    
    # As defined in move_group_python_interface.py, quaternions should be in xyzw order
    # straight up
    pos = [0.75, 0.246, 0.598]
    #ori = [0.5, 0.5, 0.5, 0.5]
    ori = [0.0, -0.7071, -0.7071, 0.0]
    move_group_interface.go_to_pose_goal(pos, ori)
    '''
    # Point arm straight up, activate the hand
    move_group_interface.go_to_joint_state(home_joints)
    hand_interface.activate()
    hand_interface.basic_mode()

    spam_pos = [0.498621045202, 0.171503284612, 0.110273849174]
    spam_pre = copy.deepcopy(spam_pos)
    # increase z for pregrasp
    spam_pre[2] += 0.1
    spam_ori = [-0.5, -0.5, 0.5, 0.5]

    pring_pos = [0.447658074228, -0.155952148686, 0.142433526017]
    pring_pre = [0.447484135281, 0, 0.224018941422]
    pring_ori = [0, 0, -1, 0]

    hardcoded_grasp(spam_pre, spam_pos, spam_ori)

    #hand_interface.wide_mode()

    #hardcoded_grasp(pring_pre, pring_pos, pring_ori)
    '''
