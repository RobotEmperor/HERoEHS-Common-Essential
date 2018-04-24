################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(heroehs_math)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_math
)

find_package(Eigen3 REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES heroehs_math
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(heroehs_math
   src/heroehs_pid_control.cpp
   src/heroehs_signal_processing.cpp
   src/fifth_order_trajectory_generate.cpp
   src/kinematics.cpp
   src/end_point_to_rad_cal.cpp
)

add_dependencies(heroehs_math ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(heroehs_math
   ${catkin_LIBRARIES}
)
