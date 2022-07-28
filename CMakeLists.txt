cmake_minimum_required(VERSION 2.8.3)
project(gps_trans_utm)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp rospy std_msgs)

catkin_package(
    INCLUDE_DIRS include

    CATKIN_DEPENDS
    std_msgs
    sensor_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

link_directories(include)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})