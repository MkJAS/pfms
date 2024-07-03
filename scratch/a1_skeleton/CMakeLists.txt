# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Change project name
project (a2_skeleton) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will create your library, add the cpp files needed
add_library(ranger_lib STATIC ranger.cpp sonar.cpp laser.cpp rangerfusion.cpp analysis.cpp)

set_target_properties(ranger_lib  PROPERTIES PUBLIC_HEADER "ranger.h;sonar.h;laser.h;rangerinterface.h;rangerfusion.h;rangerfusioninterface.h;analysis.h")

# CMake will create the cell library that you will need 
add_library(cell STATIC cell.cpp)

set_target_properties(cell PROPERTIES PUBLIC_HEADER "cell.h")

# CMake will create executable add the cpp files needed
add_executable(assignment2 main.cpp)


# Cmake will link against the libraries in final step
target_link_libraries(assignment2 ranger_lib cell)

#install(TARGETS <library_name> DESTINATION lib PUBLIC_HEADER DESTINATION include)
install(TARGETS ranger_lib cell DESTINATION lib PUBLIC_HEADER DESTINATION include)