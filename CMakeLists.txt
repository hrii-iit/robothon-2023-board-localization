cmake_minimum_required(VERSION 3.0.0)
project(hrii_board_localization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  eigen_conversions
#   hrii_gri_interface
  hrii_robothon_msgs
  hrii_trajectory_planner
  dynamic_reconfigure
  message_generation
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/board_localization_param.cfg
)

# find_package(Eigen3 REQUIRED)

# catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS
#    hrii_trajectory_planner
#    hrii_gri_interface
#    hrii_robothon_msgs
# )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen3_INCLUDE_DIRS}
# )

# catkin_install_python(PROGRAMS src/fake_perception_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



