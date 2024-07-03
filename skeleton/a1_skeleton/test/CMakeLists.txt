# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.8)

# Any project name will suffice, this has connotaions when using advanced CMake Features
set(PROJECT_NAME a2-tests)
project (${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp")
message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
add_library(student_lib ${SRC_LIST})

add_library(mock_lib mock/rangermocklaser.cpp mock/rangermocksonar.cpp ${SRC_LIST})

# Now we make the gtests

set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

add_executable(rawTests test_rawdata.cpp)
target_link_libraries(rawTests ${GTEST_LIBRARIES} pthread)
target_link_libraries(rawTests student_lib)

add_executable(fusionTests test_fusion.cpp)
target_link_libraries(fusionTests ${GTEST_LIBRARIES} pthread)
target_link_libraries(fusionTests mock_lib)
