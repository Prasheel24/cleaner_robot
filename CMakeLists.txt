cmake_minimum_required(VERSION 3.2.1)
project(cleaner_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cleaner_robot
#  CATKIN_DEPENDS genmsg geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cleanerBot src/main.cpp src/cleanerBot.cpp)
target_link_libraries(cleanerBot ${catkin_LIBRARIES})

