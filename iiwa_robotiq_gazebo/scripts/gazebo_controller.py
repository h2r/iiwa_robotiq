#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

# Some code from https://bitbucket.org/robot-learning/ll4ma_robot_control/src/master/src/ll4ma_robot_control/control_switcher.py

class IiwaRobotiqGazeboController:
    def __init__(self):
        rospy.init_node('iiwa_robotiq_gazebo_controller')

        self.last_joint_positions = None
        self.rate = rospy.Rate(100)
        self.cmd_topic = "/iiwa/PositionJointInterface_state_controller/command"
        self.state_topic = "/iiwa/joint_states"
        self.rate = rospy.Rate(100)

        self.joint_desired_publisher = rospy.Publisher(self.cmd_topic, Float64MultiArray, queue_size=1)
        self.joint_state_subscriber = rospy.Subscriber(self.state_topic, JointState, self.joint_state_callback)

        while not rospy.is_shutdown() and self.last_joint_positions is None:
            self.rate.sleep()

    def move_arm_to_joint_pos(self, desired_joints):
        trajectory = self.interpolate(self.last_joint_positions, desired_joints, n_pts=2)
        print "trajectory", trajectory
        config=None
        for config in trajectory:
            self.joint_desired_publisher.publish(Float64MultiArray(data=config))
            self.rate.sleep()
        while np.linalg.norm(np.array(desired_joints) - np.array(self.last_joint_positions)) > 0.01:
            self.joint_desired_publisher.publish(Float64MultiArray(data=config))
            self.rate.sleep()

    def interpolate(self, p1, p2, n_pts=500):
        dims = []
        for a, b in zip(p1, p2):
            dims.append(np.linspace(a, b, n_pts))
        return [pt for pt in zip(*dims)]

    def joint_state_callback(self, joint_state_msg):
        self.last_joint_positions = joint_state_msg.position

if __name__ == '__main__':
    iiwa_controller = IiwaRobotiqGazeboController()
    print "Moving 1"
    iiwa_controller.move_arm_to_joint_pos([0, 1.57, 0, 1.57, 0, -1.57, 0])
    print "Moving 2"
    iiwa_controller.move_arm_to_joint_pos([0, 0, 0, 0, 0, 0, 0])
    print "Moving 3"
    iiwa_controller.move_arm_to_joint_pos([0, 1.57, 0, 1.57, 0, -1.57, 0])
    print "Moving 4"
    iiwa_controller.move_arm_to_joint_pos([0, 0, 0, 0, 0, 0, 0])
    print "done"
