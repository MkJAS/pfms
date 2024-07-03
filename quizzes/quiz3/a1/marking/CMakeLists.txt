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

#We add the mock directory here
include_directories(mock)
add_library(mockcar mock/carmock.cpp)
#target_link_libraries(mockrangers ${OpenCV_LIBS})

# Add the unit test executable against the source code of the test
add_executable(markingTest utest.cpp)
add_executable(markingTest2 utest2.cpp)
# Linking to gtest and to your libraries
target_link_libraries(markingTest GTest::GTest GTest::Main analysis ${OpenCV_LIBS} car mockcar)
target_link_libraries(markingTest2 GTest::GTest GTest::Main analysis ${OpenCV_LIBS} car)
# We can finally add the test to a list of tests to be run with make test
add_test(Q3A1 markingTest)
add_test(Q3A1 markingTest2)
