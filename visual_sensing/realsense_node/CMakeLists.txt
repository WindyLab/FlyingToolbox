cmake_minimum_required(VERSION 3.10.0)
project(realsense_node)
set(CMAKE_BUILD_TYPE RELWITHDEBINFO)

find_package(realsense2)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs  
cv_bridge image_transport
geometry_msgs tf)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_realsense test_realsense.cc)
target_link_libraries(test_realsense realsense2::realsense2 ${catkin_LIBRARIES})
