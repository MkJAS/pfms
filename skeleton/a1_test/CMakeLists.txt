# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.8.12)

# Any project name will suffice, this has connotaions when using advanced CMake Features
set(PROJECT_NAME a1_tests)
project (${PROJECT_NAME})


message("CHECK_PURE_VIRTUAL=${CHECK_PURE_VIRTUAL}")
if( CHECK_PURE_VIRTUAL )
    message("CHECKING PURE VIRTUAL")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp")
message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
add_library(student_lib ${SRC_LIST})

#add_library(student_lib ../laser.cpp ../radar.cpp ../ranger.cpp ../rangerfusion.cpp ../rangerfusioninterface.cpp ../rangerinterface.cpp ../fusiondata.cpp)

# Uses some mock classes (sensors) with the student's fusion classes
#add_library(mock_lib mock/rangermocklaser.cpp mock/rangermockradar.cpp ../rangerfusion.cpp ../rangerfusioninterface.cpp ../rangerinterface.cpp ../laser.cpp ../radar.cpp ../ranger.cpp ../fusiondata.cpp)

add_library(mock_lib mock/rangermocklaser.cpp mock/rangermocksonar.cpp mock/mockrangerfusion.cpp ${SRC_LIST})
target_link_libraries(mock_lib ${OpenCV_LIBS})

# Now we make the gtests
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()

# Add the unit tests
if( CHECK_PURE_VIRTUAL )
    add_executable(pureVirtualTests test_pure_virtual.cpp)
    target_link_libraries(pureVirtualTests ${GTEST_LIBRARIES} pthread)
    target_link_libraries(pureVirtualTests student_lib)
endif()

# The timing should be assessed in main (approximately)
#add_executable(timingTest test_timing.cpp)
#target_link_libraries(timingTest GTest::GTest GTest::Main student_lib)
#add_test(C2.3 timingTest)

add_executable(sensorConstructors test_sensorconstructors.cpp)
target_link_libraries(sensorConstructors GTest::GTest GTest::Main student_lib)
add_test(C1.0 sensorConstructors)

add_executable(getterSetter test_gettersetter.cpp)
target_link_libraries(getterSetter GTest::GTest GTest::Main student_lib)
add_test(C1.3 getterSetter)

add_executable(sonarAreaTest test_sonararea.cpp)
target_link_libraries(sonarAreaTest GTest::GTest GTest::Main student_lib)
add_test(C2.0 sonarAreaTest)

add_executable(rawRangeTest test_rawdatarange.cpp)
target_link_libraries(rawRangeTest GTest::GTest GTest::Main student_lib)
add_test(C2.2 rawRangeTest)

add_executable(rawTests test_rawdata.cpp)
target_link_libraries(rawTests GTest::GTest GTest::Main student_lib)
add_test(C2.3 rawTests)

add_executable(laserFusionTest test_laser_fusion.cpp)
target_link_libraries(laserFusionTest GTest::GTest GTest::Main mock_lib)
add_test(C2.4 laserFusionTest)

add_executable(sonarFusionTest test_sonar_fusion.cpp)
target_link_libraries(sonarFusionTest GTest::GTest GTest::Main mock_lib)
add_test(C2.5 sonarFusionTest)

add_executable(simpleFusionTests test_simple_fusion.cpp)
target_link_libraries(simpleFusionTests GTest::GTest GTest::Main mock_lib)
add_test(C2.6 simpleFusionTests)

add_executable(advancedFusionTests test_advanced_fusion.cpp)
target_link_libraries(advancedFusionTests GTest::GTest GTest::Main mock_lib)
add_test(C2.7 advancedFusionTests)


add_executable(rangerInterface test_rangerinterface.cpp)
target_link_libraries(rangerInterface GTest::GTest GTest::Main mock_lib)
add_test(C4.0 rangerInterface)

add_executable(rangerFusionInterface test_rangerfusioninterface.cpp)
target_link_libraries(rangerFusionInterface GTest::GTest GTest::Main mock_lib)
add_test(C4.1 rangerFusionInterface)

add_executable(invokeSensorData test_invokesensordata.cpp)
target_link_libraries(invokeSensorData GTest::GTest GTest::Main mock_lib)
add_test(C4.3 invokeSensorData)

add_executable(sensorOrderingTest test_sensor_ordering.cpp)
target_link_libraries(sensorOrderingTest GTest::GTest GTest::Main mock_lib)
add_test(C4.4 sensorOrderingTest)

add_executable(angleTest test_angle.cpp)
target_link_libraries(angleTest GTest::GTest GTest::Main mock_lib)
add_test(C4.5 angleTest)

add_executable(multiSensorsTest test_multi_sensors.cpp)
target_link_libraries(multiSensorsTest GTest::GTest GTest::Main mock_lib)
add_test(C4.7 multiSensorsTest)
