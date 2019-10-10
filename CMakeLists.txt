cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)
find_package(async_comm QUIET)
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
    src/utctime.cpp
)
target_link_libraries(UBLOX ${async_comm_LIBRARIES} ${gnss_utils_LIBRARIES})
target_include_directories(UBLOX PUBLIC include ${async_comm_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

option(BUILD_TESTS "Build Tests" ON)
if (${BUILD_TESTS})
   message(STATUS "Building UBLOX parser Tests")
   find_package(GTest REQUIRED)
   find_package(Threads REQUIRED)
   add_executable(ublox_test
       src/test/test_rtcm.cpp
       src/test/test_time.cpp
       src/test/test_eph.cpp)
   target_link_libraries(ublox_test gtest gtest_main pthread UBLOX)
endif()

option(BUILD_EXAMPLES "Build Examples" ON)
if (${BUILD_EXAMPLES})
	add_executable(base src/examples/base.cpp)
	target_link_libraries(base UBLOX)

	add_executable(rover src/examples/rover.cpp)
	target_link_libraries(rover UBLOX)

	add_executable(read src/examples/read.cpp)
	target_link_libraries(read UBLOX)
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
  		DEPENDENCIES
  		std_msgs)
	catkin_package(
		LIBRARIES UBLOX
		INCLUDE_DIRS include
  		CATKIN_DEPENDS std_msgs
	)

	add_executable(ublox_ros src/ros/ublox_ros.cpp)
	target_link_libraries(ublox_ros UBLOX ${catkin_LIBRARIES})
    target_include_directories(ublox_ros PUBLIC ~/ws_rtkflight/devel/include ${catkin_INCLUDE_DIRS})
    add_dependencies(ublox_ros ${catkin_EXPORTED_TARGETS})

endif()
