cmake_minimum_required(VERSION 2.8.3)
project(sound_play)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HERoEHS-Sound
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(sound_node src/sound_node.cpp)
 target_link_libraries(sound_node ${catkin_LIBRARIES})
