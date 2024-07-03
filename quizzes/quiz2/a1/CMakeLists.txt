# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz2_a1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will now create the Makefiles that are needed to make executable "shape_line_test"
add_executable(${PROJECT_NAME}_ex main.cpp)

# We make libraries
add_library(shapes shape.cpp circle.cpp rectangle.cpp triangle.cpp)
add_library(analysis analysis.cpp line.cpp)

# CMake will link the executable "shape_line" against library "shapes"
target_link_libraries(${PROJECT_NAME}_ex shapes analysis)

##################################################################################################
# The below is our an example of our automated tets for marking
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON (or put # before lines 21 and 24 to see it in qtcreator)
# then run make as usual
# finally, tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
