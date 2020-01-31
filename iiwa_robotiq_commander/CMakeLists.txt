cmake_minimum_required(VERSION 2.8.3)
project(iiwa_robotiq_commander)

find_package(catkin REQUIRED COMPONENTS rospy robotiq_3f_gripper_articulated_msgs)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)