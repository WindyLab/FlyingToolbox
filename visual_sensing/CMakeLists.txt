cmake_minimum_required(VERSION 3.10.0)
project(visual_sensing)
set(CMAKE_BUILD_TYPE RELWITHDEBINFO)


add_subdirectory(realsense_node)
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

catkin_package( CATKIN_DEPENDS std_msgs)

add_executable(offline_calculate src/offline_calculate.cc)

## Use this on NUC
target_link_libraries(offline_calculate  ${catkin_LIBRARIES} ) #${OpenCV_LIBS}

## Use this on my computer
##target_link_libraries(offline_calculate  ${OpenCV_LIBS} ${catkin_LIBRARIES} )

add_executable(cali_camera src/cali_camera.cc src/pose_filter.cc)
target_link_libraries(cali_camera  ${catkin_LIBRARIES} realsense2::realsense2) #${OpenCV_LIBS}

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
  add_executable(realsense src/realsense.cc src/log4z.cpp src/pose_filter.cc)
  add_executable(onboard_camera_video src/onboard_camera_video.cc src/log4z.cpp)

## Use this on NUC
target_link_libraries(realsense ${catkin_LIBRARIES}) #${OpenCV_LIBS}
target_link_libraries(onboard_camera_video ${catkin_LIBRARIES}) #${OpenCV_LIBS}

## Use this on my computer
##target_link_libraries(realsense ${OpenCV_LIBS} ${catkin_LIBRARIES}) #${OpenCV_LIBS}
