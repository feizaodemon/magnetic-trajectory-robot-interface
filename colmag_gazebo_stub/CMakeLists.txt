cmake_minimum_required(VERSION 3.0.2)
project(colmag_gazebo_stub)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  sensor_msgs
  rospy
  std_msgs
  trajectory_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fr3_gazebo_visible_task_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
