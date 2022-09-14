#!/usr/bin/env python

import copy
import numpy as np
import quaternion

import mj_transforms as mjtf

# Distance from origin to gripper surface is approx. 0.052 m
_origin_to_surface = 0.052
_dist_sample_to_palm_precision = 0.0822
_dist_sample_to_palm_power = 0.019#0.014
_dist_from_sample_precision = _dist_sample_to_palm_precision + _origin_to_surface
_dist_from_sample_power = _dist_sample_to_palm_power + _origin_to_surface

def get_dist_from_sample_precision():
    return _dist_from_sample_precision

def get_dist_from_sample_power():
    return _dist_from_sample_power

def listFromPose(pose_msg):
    # Used in cloud_loader.TF_handpose2Euler
    pose_list = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.w, \
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z]
    return pose_list

# From desktop_drake_label_candidates.py
def getGripperPose(sample_point_pose, use_precision_grasp, pybullet_ordering=False):
    """
    pose: 4x4
    """
    pregrasp_offset = 0.1
    dist_to_move_hand_back = _dist_from_sample_precision if use_precision_grasp else _dist_from_sample_power

    sample_point, ori_quat = mjtf.mat2PosQuat(sample_point_pose)
    # np.quaternion: wxyz
    quaternion_rot = np.quaternion(ori_quat[0], ori_quat[1], ori_quat[2], ori_quat[3])
    movement_in_hand_frame = np.array((0, 0, -1*dist_to_move_hand_back))
    movement_in_world_frame = quaternion.as_rotation_matrix(quaternion_rot).dot(movement_in_hand_frame)
    grasp_point = sample_point + movement_in_world_frame

    pregrasp_movement_in_hand_frame = np.array((0, 0, -1*pregrasp_offset))
    pregrasp_movement_in_world_frame = quaternion.as_rotation_matrix(quaternion_rot).dot(pregrasp_movement_in_hand_frame)
    pre_grasp_point = grasp_point + pregrasp_movement_in_world_frame

    # Our frame (z=approach, x=closing) is not the same as ee frame - align. By default, pybullet approach is y, closing is x, thumb towards positive x
    # np.quaternion: wxyz
    ori = sample_point_pose[:3, :3]
    mj_to_pb_alignment = np.zeros((3, 3))
    mj_to_pb_alignment[0,0] = 1
    mj_to_pb_alignment[1,2] = -1
    mj_to_pb_alignment[2,1] = 1
    #1 0 0
    #0 0 -1
    #0 1 0
    pybullet_ori = np.matmul(ori, mj_to_pb_alignment)
    pybullet_ori_quat = mjtf.mat2Quat(pybullet_ori)
    hand_orientation = np.quaternion(pybullet_ori_quat[0], pybullet_ori_quat[1], pybullet_ori_quat[2], pybullet_ori_quat[3])

    if pybullet_ordering:
        # Return orientation in xyzw for pybullet
        return (grasp_point, (hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w))
    else:
        # Return grasp and pre-grasp positions and orientation in xyzw for execution
        return (pre_grasp_point.tolist(), grasp_point.tolist(), [hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w])