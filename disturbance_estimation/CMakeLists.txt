cmake_minimum_required(VERSION 3.0.2)
project(disturbance_estimation)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  message_generation
  serial
  mavros_msgs
)

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
  
)

include_directories(/usr/include/eigen3)

add_executable(disturbance_est
src/addBiasApplyActivation.cpp
src/elementwiseOperationInPlace.cpp
src/nn_predict_data.cpp
src/nn_predict_initialize.cpp
src/predict.cpp
src/predictForRNN.cpp
src/nn_predict.cpp
src/nn_predict_terminate.cpp
src/unaryElementwise.cpp
src/disturbance_est.cpp 
src/log4z.cpp)
add_dependencies(disturbance_est ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(disturbance_est ${catkin_LIBRARIES} ${OpenMP_LIBS})


add_executable(disturbance_est_imu
src/disturbance_est_imu.cpp 
src/log4z.cpp)
add_dependencies(disturbance_est_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(disturbance_est_imu ${catkin_LIBRARIES})

add_executable(motor_state_uart src/motor_state_uart.cpp)
add_dependencies(motor_state_uart ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_state_uart ${catkin_LIBRARIES})



