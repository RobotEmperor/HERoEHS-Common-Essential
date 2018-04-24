cmake_minimum_required(VERSION 2.8.3)
project(heroehs_manager)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robotis_framework_common
  robotis_math
  robotis_controller
  robotis_controller_msgs
  cmake_modules
  #add module package
  heroehs_base_module
  heroehs_math
  action_module
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
# INCLUDE_DIRS include
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(heroehs_manager_node src/heroehs_manager_node.cpp)
add_dependencies(heroehs_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(heroehs_manager_node ${catkin_LIBRARIES})
