cmake_minimum_required(VERSION 3.0.2)
project(colmag_ros)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  urdf
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES colmag_real_fr3_command_admission colmag_fr3_hardware_safety colmag_fr3_trajectory_safety colmag_fr3_hexagon_trajectory colmag_fr3_joint_excursion_trajectory
  CATKIN_DEPENDS actionlib control_msgs roscpp sensor_msgs std_msgs trajectory_msgs urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(colmag_real_fr3_command_admission
  src/real_fr3_command_admission.cpp
)
target_compile_features(colmag_real_fr3_command_admission PUBLIC cxx_std_11)

add_library(colmag_fr3_hardware_safety
  src/fr3_hardware_safety.cpp
)
target_compile_features(colmag_fr3_hardware_safety PUBLIC cxx_std_11)

add_library(colmag_fr3_trajectory_safety
  src/fr3_trajectory_safety.cpp
)
target_compile_features(colmag_fr3_trajectory_safety PUBLIC cxx_std_11)

add_library(colmag_fr3_hexagon_trajectory
  src/fr3_hexagon_trajectory.cpp
)

add_library(colmag_fr3_joint_excursion_trajectory
  src/fr3_joint_excursion_trajectory.cpp
)
target_compile_features(colmag_fr3_joint_excursion_trajectory PUBLIC cxx_std_11)
target_link_libraries(colmag_fr3_joint_excursion_trajectory
  colmag_fr3_trajectory_safety
)
target_compile_features(colmag_fr3_hexagon_trajectory PUBLIC cxx_std_11)
target_link_libraries(colmag_fr3_hexagon_trajectory
  colmag_fr3_trajectory_safety
)

add_executable(colmag_fr3_task_trajectory_adapter
  src/colmag_fr3_task_trajectory_adapter.cpp
)
target_compile_features(colmag_fr3_task_trajectory_adapter PUBLIC cxx_std_11)
target_link_libraries(colmag_fr3_task_trajectory_adapter
  colmag_real_fr3_command_admission
  colmag_fr3_hardware_safety
  colmag_fr3_hexagon_trajectory
  colmag_fr3_joint_excursion_trajectory
  colmag_fr3_trajectory_safety
  ${catkin_LIBRARIES}
)
add_dependencies(colmag_fr3_task_trajectory_adapter ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/mock_packet_publisher_node.py
  scripts/serial_packet_publisher_node.py
  scripts/trajectory_2d_replay_publisher.py
  scripts/trajectory_symbol_top3_recognizer_node.py
  scripts/task_dispatcher_node.py
  scripts/raw_packet_to_trajectory_2d_node.py
  scripts/magnetic_ui_state_node.py
  scripts/magnetic_trajectory_dashboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  colmag_real_fr3_command_admission
  colmag_fr3_hardware_safety
  colmag_fr3_hexagon_trajectory
  colmag_fr3_joint_excursion_trajectory
  colmag_fr3_trajectory_safety
  colmag_fr3_task_trajectory_adapter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
