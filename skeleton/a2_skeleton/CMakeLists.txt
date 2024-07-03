cmake_minimum_required(VERSION 2.8.12)
project(a2_skeleton)

#1 Importing the necessary external packages
find_package( Threads REQUIRED )
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

##################################################################
# This section find the corresponding library release

execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

#message ( STATUS "System in use is ${RELEASE_CODENAME}" )

if("${RELEASE_CODENAME}" STREQUAL "xenial")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
elseif("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
elseif("${RELEASE_CODENAME}" STREQUAL "focal")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
#elseif("${RELEASE_CODENAME}" STREQUAL "Ootpa")
#    message ( STATUS "Ootpa is in use ${RELEASE_CODENAME}" )
else("${RELEASE_CODENAME}" STREQUAL "xenial")
    message ( FATAL_ERROR "Your vesrion on Ubuntu ${RELEASE_CODENAME} is not supported" )
endif("${RELEASE_CODENAME}" STREQUAL "xenial")


# Finding and linking against the simulator library
set(SIMULATOR_LIB ${CMAKE_SOURCE_DIR}/dep/lib/${RELEASE_CODENAME})
set(SIMULATOR_INCLUDE ${CMAKE_SOURCE_DIR}/dep/include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SIMULATOR_INCLUDE}
)

message ( STATUS "Looking for libray in ${SIMULATOR_LIB}" )
link_directories(${SIMULATOR_LIB})

###################################################################################################

# Change your CMakeLists.txt from here.

# You will most likely be creating some additional libraries that you will use in
# unit testsing, there are no libraries in this example

# Here we declare a C++ executable
add_executable(${PROJECT_NAME}_example main.cpp)

# Link the library against OpenCV and simulator library
target_link_libraries(${PROJECT_NAME}_example ${OpenCV_LIBS} simulator)


#################################################################
# UNIT_TESTING SECTION, to enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
# finally, tests can be run from build directory (check the name of executables)
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
#################################################################

