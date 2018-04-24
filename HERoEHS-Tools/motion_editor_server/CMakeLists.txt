cmake_minimum_required(VERSION 2.8.3)
project(motion_editor_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  robotis_math
  robotis_controller
  robotis_framework_common
  robotis_controller_msgs
  offset_tuner_msgs
  heroehs_math
  heroehs_manager
  action_module
)

find_package(Eigen REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS}
)

add_executable(motion_editor_server 
   src/motion_editor_server.cpp
   src/main.cpp
)

add_dependencies(motion_editor_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_editor_server ${yaml_cpp_LIBRARIES} ${catkin_LIBRARIES})

