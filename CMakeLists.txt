cmake_minimum_required(VERSION 2.8.3)
project(cleaner_robot)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cleaner_robot
#  CATKIN_DEPENDS genmsg geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

## To run tests, add dependencies for gtest and rostest
##if(CATKIN_ENABLE_TESTING)
##find_package(rostest REQUIRED)
##  add_rostest_gtest(testCleaner test/testCleanerRun.launch test/main.cpp test/testCleaner.cpp)
##  target_link_libraries(testCleaner ${catkin_LIBRARIES})
##  add_dependencies(tester cleanerBot beginner_tutorials_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
##endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cleanerBot src/main.cpp src/cleanerBot.cpp)
target_link_libraries(cleanerBot ${catkin_LIBRARIES})

