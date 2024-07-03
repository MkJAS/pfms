#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

include_directories(${CMAKE_SOURCE_DIR}) # includes below directory

add_executable(posetest posetest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(posetest GTest::GTest GTest::Main)
# We can finally add the test to a list of tests to be run with make test
add_test(a2test posetest)

add_executable(associationtest associationtest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(associationtest GTest::GTest GTest::Main)
# We can finally add the test to a list of tests to be run with make test
add_test(a2test associationtest)

add_executable(bogieextrapolationtest bogieextrapolationtest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(bogieextrapolationtest GTest::GTest GTest::Main)
# We can finally add the test to a list of tests to be run with make test
add_test(a2test bogieextrapolationtest)
