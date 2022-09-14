#!/home/mcorsaro/kuka_ws/virtualenvs/tf-1.14/bin/python

# This script can be used as a basis for your scipt that uses move_group_python_interface and robotiq_commander from iiwa_robotiq
# I recommend placing it in a new ROS package in kuka_ws/src, and create a new GitHub repo for the new package

import sys
import random

import numpy as np
import quaternion

import struct

import gripper_pose

# ros
import rospy
import ros_numpy
import tf
import actionlib

# iiwa_robotiq interfaces
import iiwa_robotiq_commander.move_group_python_interface as moveit_interface
import iiwa_robotiq_commander.robotiq_commander as robotiq_commander

# messages
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

import math
import copy
import time
from operator import mul

from collections import deque

# Hard-coded positions
home_joints = [0]*7

class SingleCloudSubscriber:
    def __init__(self):
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.cloud_callback, queue_size=None)
        rospy.Subscriber("/camera/depth/image", Image, self.depth_img_callback, queue_size=None)
        self.latest_cloud = None
        self.latest_depth_image = None

    def cloud_callback(self, pointcloud2):
        self.latest_cloud = pointcloud2

    def depth_img_callback(self, depth_img):
        self.latest_depth_image = depth_img

class CameraInfoSubscriber:
    def __init__(self):
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cam_info_callback, queue_size=None)
        self.latest_cam_info = None

    def cam_info_callback(self, cam_info):
        self.latest_cam_info = cam_info

def pose_msg_from_position_and_orientation(pose_list):
    position, orientation = pose_list
    pose_msg = Pose()
    pose_msg.position.x = position[0]
    pose_msg.position.y = position[1]
    pose_msg.position.z = position[2]
    pose_msg.orientation.x = orientation[0]
    pose_msg.orientation.y = orientation[1]
    pose_msg.orientation.z = orientation[2]
    pose_msg.orientation.w = orientation[3]
    return pose_msg

def make_robot_state_msg(joints, operating_mode):
    # Create message to use as initial planning configuration. Note: Assumes the hand is open..
    if len(joints) != 7 or operating_mode not in ["basic", "wide", "pincher"]:
        raise rospy.ROSException("CANNOT GENERATE STATE MSG WITH {} JOINTS AND OPERATING MODE {}.".format(len(joints), operating_mode))
    # Joints = list of joint values
    # operating_mode = hand operating mode (basic, wide, pincher)
    joint_state = JointState()
    joint_state.header = Header()
    #joint_state.header.stamp = rospy.Time.now()
    joint_state.header.frame_id = "/world"
    joint_state.name = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7", "palm_finger_1_joint", "finger_1_joint_1", "finger_1_joint_2", "finger_1_joint_3", "palm_finger_2_joint", "finger_2_joint_1", "finger_2_joint_2", "finger_2_joint_3", "finger_middle_joint_1", "finger_middle_joint_2", "finger_middle_joint_3"]
    # Basic by default
    hand_joint_pos = [-0.016212169496302907, 0.04952961390794719, 0.0, -0.05235987755982989, 0.016212169496302907, 0.04952961390794719, 0.0, -0.05235987755982989, 0.04952961390794719, 0.0, -0.05235987755982989]
    if operating_mode == "wide":
        hand_joint_pos = [0.17833386445933172,   0.04952961390794719, 0.0, -0.05235987755982989, -0.17833386445933172, 0.04952961390794719, 0.0, -0.05235987755982989, 0.04952961390794719, 0.0, -0.05235987755982989]
    elif operating_mode == "pincher":
        hand_joint_pos = [-0.15785533456926493,  0.04952961390794719, 0.0, -0.05235987755982989, 0.15785533456926493,  0.04952961390794719, 0.0, -0.05235987755982989, 0.04952961390794719, 0.0, -0.05235987755982989]
    joint_state.position = joints + hand_joint_pos
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    return moveit_robot_state

def array_msg_to_np_mat(array_msg):
    array_dims = array_msg.layout.dim
    array_sizes = [dim.size for dim in array_dims]
    # (num_candidates x num_grasp_types) array, with values from 0 to 1 or also -1 if collision/too few points to evaluate
    np_mat = np.array(array_msg.data).reshape(array_sizes)
    return np_mat

def move_joints_to_within_soft_limits(joint_list):
    if len(joint_list) != 7:
        raise rospy.ROSException("JOINT STATE CONTAINS {} JOINTS.".format(len(joint_list)))
    # TODO: use MoveIt interface, I can't find a way to get joint limits in Python..
    # These limits are copied from iiwa7.xacro
    lower_limits = [-168.*math.pi/180., -118.*math.pi/180., -168.*math.pi/180., \
        -118.*math.pi/180., -168.*math.pi/180., -118.*math.pi/180., -173.*math.pi/180.]
    upper_limits = [168.*math.pi/180., 118.*math.pi/180., 168.*math.pi/180., \
        118.*math.pi/180., 168.*math.pi/180., 118.*math.pi/180., 173.*math.pi/180.]
    new_joints = copy.deepcopy(joint_list)
    for i in range(len(new_joints)):
        if new_joints[i] < lower_limits[i]:
            new_joints[i] = lower_limits[i]
            rospy.logwarn("Setting joint {} to minimum {:.3f}.".format(i, lower_limits[i]))
        if new_joints[i] > upper_limits[i]:
            new_joints[i] = upper_limits[i]
            rospy.logwarn("Setting joint {} to maximum {:.3f}.".format(i, upper_limits[i]))
    return new_joints

def findMinimumDistanceFromPointToPoints(sampled_pose, poses):
    min_dist = 9999999999999
    sampled_point = sampled_pose[:3, 3]
    for pose in poses:
        pose_point = pose[:3, 3]
        dist = np.linalg.norm(sampled_point-pose_point)
        if dist < min_dist:
            min_dist = dist
    return min_dist

if __name__ == '__main__':
    use_real_robot = True
    rospy.init_node('execute_selected_grasps')

    grasp_types = ["wide_power", "wide_precision", "basic_power", "basic_precision", "pincher"]
    corresponding_operating_modes = ["wide", "wide", "basic", "basic", "pincher"]
    corresponding_grasp_is_precision = [False, True, False, True, True]
    # Default 5, also 1 or 2
    num_types = 5

    grasp_pose_marker_pub = rospy.Publisher('grasp_pose_marker', Marker, queue_size=1)
    pregrasp_pose_marker_pub = rospy.Publisher('pregrasp_pose_marker', Marker, queue_size=1)
    tf_broadcaster = tf.TransformBroadcaster()

    # Interfaces to interact with the arm and hand
    move_group_interface = moveit_interface.MoveGroupPythonInterface("palm_link")
    # TODO: set planning time in moveit config - allowed_planning_time didn't seem to have the same effect
    # TODO: create a iiwa_robotiq script that generates and saves a PRM
    #move_group_interface.group.set_planning_time(1800)
    # When using a pregenerated PRM, use as really low planning time (5 ms should work, higher is fine)
    move_group_interface.group.set_planning_time(1.0)
    hand_interface = robotiq_commander.RobotiqInterface()
    # Move to the upright position
    move_group_interface.go_to_joint_state(home_joints)
    # Activate the gripper, which involves waiting for 20 seconds, even if the gripper had been activated previously
    hand_interface.activate()
    # Basic mode sets the two fingers parallel to each other
    hand_interface.basic_mode(delay=1)

    # Subscribe to point cloud published by Structure Sensor
    cloud_sub = SingleCloudSubscriber()
    # Subscribe to Structure Sensor info
    cam_info_sub = CameraInfoSubscriber()
    # Listen to TF to get camera pose in the world frame
    tf_sub = tf.TransformListener()

    hand_interface.open(delay=1)
    # Move to a set of joints to capture a point cloud from. I usually use the interactive
    #     moveit window (RViz==true in screen 2) to find a good pose that points the camera
    #     towards the workspace offline before running this script, then hard-code the joint positions.
    move_group_interface.go_to_joint_state(home_joints)
    rospy.loginfo("Looking up transform 1")
    # [position, (x, y, z, w) orientation]
    world_to_cloud_tf = tf_sub.lookupTransform('/world', '/camera_depth_optical_frame', rospy.Time(0))
    new_cloud = cloud_sub.latest_cloud
    new_depth_image = cloud_sub.latest_depth_image
    rospy.loginfo("Now fetching latest clouds 1 with h{}w{} points.".format(new_cloud.height, new_cloud.width))
    cam_info = cam_info_sub.latest_cam_info
    # Perform point cloud processing here

    # Use the point cloud or depth image to select a grasp pose (in the world frame)

    # An example of adding an obstacle to the MoveIt environment. In this case, it's the handle for the cabinet.
    move_group_interface.add_box_obstacle((-0.3175, -0.18415, -0.119), (0., 0., 0., 1.), (0.0889, 0.0889, 0.4572), name="handle", frame_id="cabinet")

    # Execute a basic precision grasp at a sampled point from the cloud, with an assigned orientation.
    grasp_type_index = 3
    # pre_grasp_position is a distance away from the object, then the gripper approaches along the approach direction to grasp_position
    # sampled_point_pose is undefined here. It is a 4x4 homogeneous transformation matrix centered at a point from the point cloud in the world frame
    sampled_point_pose = DEFINE_HERE
    pre_grasp_position, grasp_position, orientation = gripper_pose.getGripperPose(sampled_point_pose, corresponding_grasp_is_precision[grasp_type_index])

    # Visualize grasp poses in rviz
    tf_broadcaster.sendTransform(pre_grasp_position, orientation, rospy.Time.now(), "/pregrasp_pose", "/world")
    tf_broadcaster.sendTransform(grasp_position, orientation, rospy.Time.now(), "/grasp_pose", "/world")
    
    # Plan a path from the current joint state to the pre-grasp
    pre_grasp_path = move_group_interface.get_plan_to_pose_goal(pose_goal_position=pre_grasp_position, pose_goal_quat=orientation)
    # If the pregrasp is feasible 
    if len(pre_grasp_path.joint_trajectory.points) > 0:
        # Remove the cabinet handle when moving from the pregrasp to the grasp
        move_group_interface.remove_obstacle("handle")
        pregrasp_joints = list(pre_grasp_path.joint_trajectory.points[-1].positions)
        pre_grasp_pose_msg = make_robot_state_msg(pregrasp_joints, corresponding_operating_modes[0])
        #grasp_path = move_group_interface.get_plan_to_pose_goal(pose_goal_position=grasp_position, pose_goal_quat=orientation, start_state=pre_grasp_pose_msg)
        grasp_path_cartesian, grasp_path_fraction = move_group_interface.get_cartesian_plan(pose_goal_position=grasp_position, pose_goal_quat=orientation, start_state=pre_grasp_pose_msg)
        if grasp_path_fraction > 0.7:
            # If the arm can move most of the way to the grasp pose, execute the pregrasp and grasp
            grasp_joints = list(grasp_path_cartesian.joint_trajectory.points[-1].positions)
            grasp_pose_msg = make_robot_state_msg(grasp_joints, corresponding_operating_modes[0])

            # When executing the pregrasp, add the handle model back in
            move_group_interface.add_box_obstacle((-0.3175, -0.18415, -0.119), (0., 0., 0., 1.), (0.0889, 0.0889, 0.4572), name="handle", frame_id="cabinet")
            # Replans, could also execute the old computed path
            executed, _, _ = move_group_interface.go_to_joint_state(pregrasp_joints)
            move_group_interface.remove_obstacle("handle")

            # Now execute the grasp
            if executed:
                executed, _, grasp_fraction, grasp_plan_length = move_group_interface.go_to_cartesian_goal(grasp_position, orientation, delay=1)

                if executed and grasp_fraction > 0.7:
                    # Close the gripper
                    hand_interface.close(delay=1)

                    # Manipulate the object

                    # Open the gripper
                    hand_interface.open(delay=1)
