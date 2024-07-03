# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Change project name
project (a2_skeleton) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will create your library, add the cpp files needed
#add_library(ranger )

# CMake will create the cell library that you will need 
add_library(cell cell.cpp)

# CMake will create executable add the cpp files needed
#add_executable(assignment2 )


# Cmake will link against the libraries in final step
#target_link_libraries(assignment2 ranger cell)
