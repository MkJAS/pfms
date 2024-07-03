cmake_minimum_required(VERSION 2.8.12)
## This is the name of your project
## STUDENTS !!!
## REPLACE below project name to be a3_studentID and change line 3 in package.xml
project(a3_skeleton)
# FOR EXAMPLE
# project(a3_9847421)
# in package.xml replace the a3_skeleton with your ID, for example
# <name>a3_9847421</name>

find_package(catkin REQUIRED COMPONENTS
  project_setup # Needed for the services
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  rosbag # Needed for rosbag manipulation
  roslib # Needed for ros::package::getPath
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

## We tell then system we have a catkin_package, no dependencies going forward
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###############
## Libraries ##
###############

## STUDENTS !!!
## Declare C++ libraries here
## Example provided with grid-processing (which you will most liekly use
## It is expected you would have more libraries for your work and ADD MORE lines
add_library(${PROJECT_NAME}_grid_processing src/grid_processing.cpp)
# FOR EXAMPLE
#add_library(${PROJECT_NAME}_my_library src/my_library.cpp)


###############
## Executable ##
###############

## Declare C++ executable
## STUDENTS !!!
## if you do have more cpp files that are not part of library add them here
## this is unlikely, so you would not need to change, otherwise add to end
add_executable(${PROJECT_NAME}_ex src/main.cpp src/sample.cpp)

## Add cmake target dependencies of the executable, enforces generating messages
## VERY VERY VERY HIGHLY unlikely to change
add_dependencies(${PROJECT_NAME}_ex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
## STUDENTS !!!
## It is expected you would have more libraries for your work and would need to REPLACE the below line
target_link_libraries(${PROJECT_NAME}_ex ${catkin_LIBRARIES} ${PROJECT_NAME}_grid_processing)
# FOR EXAMPLE
#target_link_libraries(${PROJECT_NAME}_ex ${catkin_LIBRARIES} ${PROJECT_NAME}_grid_processing ${PROJECT_NAME}_my_library)


#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries
## STUDENTS !!!
## We provide example with grid_processing ADD MORE of same syntax in below lines for your libraries
catkin_add_gtest(${PROJECT_NAME}_grid_processing_test test/grid_processing_utest.cpp )
if(TARGET ${PROJECT_NAME}_grid_processing)
   target_link_libraries(${PROJECT_NAME}_grid_processing_test ${catkin_LIBRARIES} ${PROJECT_NAME}_grid_processing )
endif()

# FOR EXAMPLE
#catkin_add_gtest(${PROJECT_NAME}_my_library_test test/my_library_utest.cpp )
#if(TARGET ${PROJECT_NAME}_my_library)
#   target_link_libraries(${PROJECT_NAME}_my_library_test ${catkin_LIBRARIES} ${PROJECT_NAME}_my_library )
#endif()


