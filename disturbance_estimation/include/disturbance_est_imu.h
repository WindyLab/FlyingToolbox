
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "disturbance_estimation/motor_state.h"


using namespace std;
using namespace Eigen;

#define pi 3.14159

class disturbance_est_imu {
 public:
  disturbance_est_imu(ros::NodeHandle& n_);
  ~disturbance_est_imu();
  void saveGroundThruth();
  
 private:
  ros::Subscriber State_sub_;
  ros::Subscriber Att_sub_;
  ros::Subscriber Quadrotor_sub_;
  ros::Subscriber Hexarotor_sub_;
  ros::Subscriber IMU_acc_sub_;
  ros::Subscriber Motor_sub_;
  ros::Subscriber RC_In_sub_;
  ros::Subscriber attitude_sub; 
  ros::Publisher Disturbance_force_pub_;

  double para_thrust_[4];

  void Calculate_Disturbance();  // save in Disturbance_
  void Calculate_Thrust();  // obtain the total thrust save in T_
  void State_CallBack(const mavros_msgs::State::ConstPtr&);
  void Att_CallBack(const sensor_msgs::Imu::ConstPtr&);
  void Quadrotor_Callback(const geometry_msgs::TransformStamped::ConstPtr&);
  void Hexarotor_CallBack(const geometry_msgs::TransformStamped::ConstPtr&);
  void IMU_Acc_CallBack(const geometry_msgs::Vector3::ConstPtr&);
  void Motor_state_CallBack(const disturbance_estimation::motor_state::ConstPtr&);
  void RC_In_CallBack(const mavros_msgs::RCIn::ConstPtr&);
  void att_obtain(const sensor_msgs::Imu::ConstPtr& msg);

  bool is_arm_;

  double lower_uav_psi_;
  uint8_t Docking_mode_count_;
  Eigen::Vector3d IMU_Acc_;
  Eigen::Vector3d IMU_Acc_Bias_; 
  disturbance_estimation::motor_state motor_state_;
  geometry_msgs::TransformStamped Quadrotor_data_;
  geometry_msgs::TransformStamped Hexarotor_data_;
  Eigen::Vector3d Disturbance_;
  Eigen::Vector3d F_;
  Eigen::Vector3d G_;
  Eigen::Vector3d Re3_;  // rotation matrix time e3, e3=[0,0,1]
  float T_;              // total thrust
  double mass_aerbox_;  // mass(kg)
  bool is_calibration_;
};
