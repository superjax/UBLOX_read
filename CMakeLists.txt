cmake_minimum_required(VERSION 2.8.3)
project(ublox)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(async_comm)
if (NOT ${async_comm_FOUND})
	add_subdirectory(lib/async_comm)
	set(async_comm_LIBRARIES async_comm)
	set(async_comm_INCLUDE_DIRS lib/async_comm/include)
endif()

add_library(UBLOX
    src/ublox.cpp
    src/parsers/ubx.cpp
    src/parsers/rtcm.cpp
    src/parsers/nmea.cpp
)
target_link_libraries(UBLOX ${async_comm_LIBRARIES})
target_include_directories(UBLOX PUBLIC include ${async_comm_INCLUDE_DIRS})


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
    message(WARNING "BUILDING UBLOX ROS NODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	find_package(catkin REQUIRED COMPONENTS
		roscpp
		sensor_msgs
		message_generation)

	add_message_files(
		DIRECTORY msg
		FILES
		GNSS.msg
		PositionVelocityTime.msg
		RelPos.msg)
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
    target_include_directories(ublox_ros PUBLIC ${catkin_INCLUDE_DIRS})
    add_dependencies(ublox_ros ${catkin_EXPORTED_TARGETS})

endif()
