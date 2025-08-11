cmake_minimum_required(VERSION 3.0.2)
project(disturbance_estimation)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  message_generation
  serial
  mavros_msgs
)
find_package(gazebo REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

add_message_files(
  FILES 
  motor_state.msg
)
generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  
)

include_directories(/usr/include/eigen3)

add_executable(disturbance_est
src/disturbance_est.cpp
src/log4z.cpp
)

add_dependencies(disturbance_est disturbance_estimation_gencpp)

target_link_libraries(disturbance_est ${catkin_LIBRARIES})


# #uncomment the following 4 lines to use the Eigen library
# find_package(cmake_modules REQUIRED)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# add_definitions(${EIGEN_DEFINITIONS})