cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp std_msgs)


## Generate services in the 'srv' folder
add_service_files(FILES
  traj_out_msg.srv
  traj_solver_msg.srv
  switch2moving_msg.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


link_directories(
 lib
  ${catkin_LIB_DIRS}
)

add_executable(traj_solver
 src/bezier_base.cpp
 src/trajectory_solver.cpp
 src/auto_dock.cpp
)
add_dependencies(traj_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_solver
 ${catkin_LIBRARIES}
 osqp  # library name
)


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})



