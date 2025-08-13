cmake_minimum_required(VERSION 3.10.0)
project(visual_sensing)
set(CMAKE_BUILD_TYPE RELWITHDEBINFO)

find_package(OpenCV REQUIRED) 
message("=============",${OpenCV_LIBS})
message("find ${OpenCV_VERSION}")

find_package(realsense2)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs  
geometry_msgs cv_bridge image_transport tf message_generation)



add_message_files(
  FILES
  toolbox.msg
  toolbox_array.msg

)

generate_messages(
  DEPENDENCIES
  
  std_msgs
  sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package( CATKIN_DEPENDS std_msgs)

add_executable(cali_camera src/cali_camera.cc src/pose_filter.cc)
target_link_libraries(cali_camera  ${catkin_LIBRARIES} realsense2::realsense2) #${OpenCV_LIBS}


add_executable(realsense src/realsense.cc src/log4z.cpp src/pose_filter.cc)
add_dependencies(realsense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(realsense ${catkin_LIBRARIES}) #${OpenCV_LIBS}


add_executable(connect_realsense src/connect_realsense.cc)
target_link_libraries(connect_realsense realsense2::realsense2 ${catkin_LIBRARIES})