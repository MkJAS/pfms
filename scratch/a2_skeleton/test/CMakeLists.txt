#######################################################################

include_directories(..)
include_directories(..)

find_package( Threads REQUIRED )
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(SIMULATOR_LIB .. /dep/lib/${RELEASE_CODENAME})
set(SIMULATOR_INCLUDE .. /dep/include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SIMULATOR_INCLUDE}
)

link_directories(${SIMULATOR_LIB})

# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(
 ${GTEST_INCLUDE_DIRS}
 ${SIMULATOR_INCLUDE} #This enabled us to include simulator
 ${CMAKE_SOURCE_DIR}  #This enabled us to include files that are part of project (such as tf.h)
)

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

include_directories(${CMAKE_SOURCE_DIR}) # includes below directory

add_executable(posetest posetest.cpp mockbogiepos.cpp)
# Linking to gtest and to your libraries
target_link_libraries(posetest GTest::GTest GTest::Main a2_lib simulator ${OpenCV_LIBS})
# We can finally add the test to a list of tests to be run with make test
add_test(a2test posetest)

add_executable(associationtest associationtest.cpp mockbogiepos.cpp)
# Linking to gtest and to your libraries
target_link_libraries(associationtest GTest::GTest GTest::Main a2_lib simulator ${OpenCV_LIBS})
# We can finally add the test to a list of tests to be run with make test
add_test(a2test associationtest)

add_executable(bogieextrapolationtest bogieextrapolationtest.cpp mockbogieposAdv.cpp mockbogiepos.cpp)
# Linking to gtest and to your libraries
target_link_libraries(bogieextrapolationtest GTest::GTest GTest::Main a2_lib simulator ${OpenCV_LIBS})
# We can finally add the test to a list of tests to be run with make test
add_test(a2test bogieextrapolationtest)
