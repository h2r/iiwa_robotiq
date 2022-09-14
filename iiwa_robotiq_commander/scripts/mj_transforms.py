'''
Copyright, 2021, Matt Corsaro, matthew_corsaro@brown.edu
'''

import numpy as np
import scipy
from scipy.spatial.transform import Rotation


# mujoco to ROS
def wxyz2xyzw(wxyz):
    return wxyz[1:] + [wxyz[0]]
# ROS to mujoco
def xyzw2wxyz(xyzw):
    return [xyzw[-1]] + list(xyzw[:-1])

"""
Generates numpy rotation matrix from quaternion

@param rot_vec: rotation vector of length 3

@return np_rot_mat: 3x3 rotation matrix as numpy array
"""
def rotVec2Mat(rot_vec):
    if len(rot_vec) != 3:
        print("Rotation vector", rot_vec, "invalid when generating transformation matrix.")
        raise ValueError
    rotvec_scipy = Rotation.from_rotvec(rot_vec)
    if scipy.__version__ < '1.4':
        return rotvec_scipy.as_dcm()
    else:
        return rotvec_scipy.as_matrix()

"""
Generates numpy rotation matrix from quaternion

@param quat: w-x-y-z quaternion rotation tuple

@return np_rot_mat: 3x3 rotation matrix as numpy array
"""
def quat2Mat(quat):
    if len(quat) != 4:
        print("Quaternion", quat, "invalid when generating transformation matrix.")
        raise ValueError
    # MuJoCo and Open3D use wxyz, scipy uses xyzw
    quat_xyzw = quat[1:] + [quat[0]]
    quat_scipy = Rotation(quat_xyzw)
    if scipy.__version__ < '1.4':
        return quat_scipy.as_dcm()
    else:
        return quat_scipy.as_matrix()

    # Note that the following code snippet can be used to generate the 3x3
    #    rotation matrix, we don't use it because this file should not depend
    #    on mujoco.
    '''
    from mujoco_py import functions
    res = np.zeros(9)
    functions.mju_quat2Mat(res, camera_quat)
    res = res.reshape(3,3)
    '''

"""
Generates quaternion list (wxyz) from numpy rotation matrix

@param np_rot_mat: 3x3 rotation matrix as numpy array

@return quat: w-x-y-z quaternion rotation list
"""
def mat2Quat(np_rot_mat):
    rot = None
    if scipy.__version__ < '1.4':
        rot = Rotation.from_dcm(np_rot_mat)
    else:
        rot = Rotation.from_matrix(np_rot_mat)
    quat_xyzw = rot.as_quat()
    quat_wxyz = [quat_xyzw[3]] + list(quat_xyzw)[:3]
    return quat_wxyz

"""
Generates 4x4 transformation matrix from position and euler angles (rpy).

@param pos: position list (len 3)
@param euler: rotation angle list (len 3)

@return t_mat: 4x4 transformation matrix
"""
def posEuler2Mat(pos, euler):
    t_mat = np.eye(4)
    t_mat[:3, 3] = np.array(pos)

    rot = Rotation.from_euler('xyz', euler, degrees=False)
    rot_mat = None
    if scipy.__version__ < '1.4':
        rot_mat = rot.as_dcm()
    else:
        rot_mat = rot.as_matrix()
    t_mat[:3, :3] = rot_mat
    return t_mat

"""
Generates position list and quaternion list (wxyz) from numpy transformation matrix

@param np_mat: 4x4 transformation matrix as numpy array

@return pos:  x-y-z position list
@return quat: w-x-y-z quaternion rotation list
"""
def mat2PosQuat(np_mat):
    pos = list(np_mat[:3,3])
    quat_wxyz = mat2Quat(np_mat[:3,:3])
    return (pos, quat_wxyz)

"""
Generates numpy rotation matrix from rotation matrix as list len(9)

@param rot_mat_arr: rotation matrix in list len(9) (row 0, row 1, row 2)

@return np_rot_mat: 3x3 rotation matrix as numpy array
"""
def rotMatList2NPRotMat(rot_mat_arr):
    np_rot_arr = np.array(rot_mat_arr)
    np_rot_mat = np_rot_arr.reshape((3, 3))
    return np_rot_mat

"""
Generates numpy transformation matrix from position list len(3) and
    numpy rotation matrix

@param pos:     list len(3) containing position
@param rot_mat: 3x3 rotation matrix as numpy array

@return t_mat:  4x4 transformation matrix as numpy array
"""
def posRotMat2Mat(pos, rot_mat):
    t_mat = np.eye(4)
    t_mat[:3, :3] = rot_mat
    t_mat[:3, 3] = np.array(pos)
    return t_mat

"""
Generates transformation matrix from numpy array len(7) of position list and
    quaternion

@param np_grasp:  numpy array len(7) with position followed by w-x-y-z quat

@return t_mat:  4x4 transformation matrix as numpy array
"""
def npGraspArr2Mat(np_grasp):
    grasp_position = np_grasp[:3].tolist()
    grasp_orientation = (np_grasp[3:]/np.linalg.norm(np_grasp[3:])).tolist()
    t_mat = posRotMat2Mat(grasp_position, quat2Mat(grasp_orientation))
    return t_mat

# Returns difference in radians between two quaternions
def quatDiff(quat1, quat2):
    cos_angle = (np.trace(np.matmul(np.transpose(quat2Mat(quat1)), quat2Mat(quat2)))-1)/2
    if cos_angle > 1.0001 or cos_angle < -1.0001:
        print("Cosine of angle is outside acceptable range:", cos_angle)
    if cos_angle > 1:
        return 0.0
    elif cos_angle < -1:
        return math.pi
    return np.arccos(cos_angle)
