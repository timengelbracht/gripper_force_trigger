cmake_minimum_required(VERSION 3.0.2)
project(gripper_force_trigger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gripper_force_trigger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
