# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.12)

# Add project name
project (week08_solutions)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

#We add the mock directory here
include_directories(mock)
add_library(mockrangers mock/rangermocklaser.cpp mock/rangermocksonar.cpp mock/mockrangerfusion.cpp)
target_link_libraries(mockrangers ${OpenCV_LIBS})

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(A2_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${A2_LIB_HOME}/include)
link_directories(${A2_LIB_HOME}/lib)

add_executable(ex01 main.cpp)
# Link to the library, this presumes you have built three libraries (each with name below)
# If you only had one library then you can add a single library name
target_link_libraries(ex01 cell ranger_lib)

##################################################################################################
# The below is enabling testing

set(BUILD_TESTS ON ) # This line enables tests without needing to pass swicth

if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
