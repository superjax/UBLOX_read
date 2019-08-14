# Copyright (c) 2019 James Jackson
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)
find_package(async_comm REQUIRED)
if (NOT ${async_comm_FOUND})
	add_subdirectory(lib/async_comm)
	set(async_comm_LIBRARIES async_comm)
	set(async_comm_INCLUDE_DIRS lib/async_comm/include)
endif()

add_library(UBLOX
    src/ublox.cpp
    src/parsers/ubx.cpp
    src/parsers/rtcm.cpp
    src/parsers/nav.cpp
	src/bit_tools.cpp
	src/utctime.cpp
	src/crc.cpp
)
target_include_directories(UBLOX PUBLIC include)

option(BUILD_TESTS "Build Tests" ON)
if (${BUILD_TESTS})
    message(STATUS "Building Tests")
    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)
    add_executable(ublox_test
	src/test/test_rtcm.cpp
	src/test/test_time.cpp
	src/test/test_bit_tools.cpp
	src/test/test_eph.cpp)
	target_link_libraries(ublox_test gtest gtest_main pthread UBLOX)
	target_include_directories(ublox_test PRIVATE src/test)
endif()
	
option(BUILD_EXAMPLES "Build Examples" ON)
if (${BUILD_EXAMPLES})
	message(STATUS "Building Examples")
	add_executable(base src/examples/base.cpp)
	target_link_libraries(base UBLOX ${async_comm_LIBRARIES})
	target_include_directories(base PRIVATE ${async_comm_INCLUDE_DIRS})
	
	add_executable(rover src/examples/rover.cpp)
	target_link_libraries(rover UBLOX ${async_comm_LIBRARIES})
	target_include_directories(rover PRIVATE ${async_comm_INCLUDE_DIRS})
	
	add_executable(read_serial src/examples/read_serial.cpp)
	target_link_libraries(read_serial UBLOX  ${async_comm_LIBRARIES})
	target_include_directories(read_serial PRIVATE ${async_comm_INCLUDE_DIRS})

	add_executable(read_log src/examples/read_log.cpp)
	target_link_libraries(read_log UBLOX  ${async_comm_LIBRARIES})
	target_include_directories(read_log PRIVATE ${async_comm_INCLUDE_DIRS})
endif()

option(BUILD_ROS "Build ROS Node" ON)
if (${BUILD_ROS})
    message(STATUS "BUILDING UBLOX ROS PACKAGE")
	find_package(catkin REQUIRED COMPONENTS
		rosbag
		roscpp
		sensor_msgs
		message_generation)

	add_message_files(
		DIRECTORY msg
		FILES
		PosVelEcef.msg
		PositionVelocityTime.msg
		RelPos.msg
		SurveyStatus.msg
		Ephemeris.msg
		GlonassEphemeris.msg
		Observation.msg
		ObsVec.msg)

	generate_messages(
  		DEPENDENCIES std_msgs)
		  
	catkin_package(
		LIBRARIES UBLOX
		INCLUDE_DIRS include
  		CATKIN_DEPENDS std_msgs
	)

	add_executable(ublox_ros src/ros/ublox_ros.cpp)
	target_link_libraries(ublox_ros UBLOX ${catkin_LIBRARIES} ${async_comm_LIBRARIES})
    target_include_directories(ublox_ros PUBLIC ${catkin_INCLUDE_DIRS} PRIVATE ${async_comm_INCLUDE_DIRS})
    add_dependencies(ublox_ros ${catkin_EXPORTED_TARGETS})

endif()
