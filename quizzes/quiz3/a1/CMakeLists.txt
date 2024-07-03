# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.8.12)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz3_a1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(CAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${CAR_LIB_HOME}/include)
message ( STATUS "Looking for libray in ${CAR_LIB_HOME}" )
link_directories(${CAR_LIB_HOME}/lib)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)


add_library(displayrace display_race.cpp)
target_link_libraries( displayrace ${OpenCV_LIBS})


add_library(analysis analysis.cpp)
target_link_libraries(analysis car displayrace)


add_executable(racing_ex main.cpp)
target_link_libraries( racing_ex ${OpenCV_LIBS} car analysis displayrace)

##################################################################################################
# The below is our an example of our automated tets for marking
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
# finally, tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
