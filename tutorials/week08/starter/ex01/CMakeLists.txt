# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.12)

# Add project name
project (week08_ex01)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#We add the mock directory here
include_directories(mock)
add_library(mockrangers mock/rangermocklaser.cpp mock/rangermocksonar.cpp)


# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(A2_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${A2_LIB_HOME}/include)
link_directories(${A2_LIB_HOME}/lib)

add_executable(ex01 main.cpp)
# Link to the library, this presumes you have built three libraries (each with name below)
# If you only had one library then you can add a single library name
target_link_libraries(ex01 cell ranger_lib)

##################################################################################################
# The below is our an example of our automated tets for marking
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing we instaed of passing swicth set it to ON
set(BUILD_TESTS ON )

if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
