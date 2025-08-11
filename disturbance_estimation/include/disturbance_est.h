#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
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

#include "log4z.h"

using namespace std;
using namespace Eigen;

#define pi 3.14159

class disturbance_est {
 public:
  disturbance_est(ros::NodeHandle& n_);
  ~disturbance_est();
  void saveGroundThruth();
  void sover_estimate();
  
 private:
  ros::Subscriber State_sub_;
  ros::Subscriber Quadrotor_Edison_sub_;
  ros::Subscriber Quadrotor_Turing_sub_;
  ros::Subscriber Hexarotor_sub_;
  ros::Subscriber RC_In_sub_;
  ros::Subscriber attitude_sub;
  ros::Subscriber turth_force_sub_;
  ros::Publisher Disturbance_torque_pub_;
  ros::Publisher Disturbance_force_pub_;


  void airflow_model(double r, double dz,double& vel);
  void Calculate_Disturbance();  //  save in Disturbance_
  void Calculate_Thrust();  //  obtain the total thrust save in T_
  void State_CallBack(const mavros_msgs::State::ConstPtr&);
  void Edison_Callback(const geometry_msgs::TransformStamped::ConstPtr&);
  void Turing_Callback(const geometry_msgs::TransformStamped::ConstPtr&);
  void Hexarotor_CallBack(const geometry_msgs::TransformStamped::ConstPtr&);
  void att_obtain(const sensor_msgs::Imu::ConstPtr& msg);
  void RC_In_CallBack(const mavros_msgs::RCIn::ConstPtr&);
  void PINN_estimate(Eigen::Vector3d upper_uav_pos,Eigen::Vector3d lower_uav_pos,geometry_msgs::Vector3& estimate_force,geometry_msgs::Vector3& estimate_torque);
  bool vicon_data_is_avilable(geometry_msgs::TransformStamped in_data);

  bool is_arm_;
  double lower_uav_psi_;
  uint8_t Docking_mode_count_;
  geometry_msgs::TransformStamped Quadrotor_Turing_data_;
  geometry_msgs::TransformStamped Quadrotor_Edison_data_;
  geometry_msgs::TransformStamped Hexarotor_data_;
  Vector3d Truth_Force_data_;
  bool UAV_position_update =  true;
};
