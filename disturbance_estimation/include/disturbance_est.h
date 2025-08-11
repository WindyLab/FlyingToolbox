
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "log4z.h"

using namespace std;

class disturbance_est {
 public:
  disturbance_est(ros::NodeHandle& n_);
  ~disturbance_est();
  void State_CallBack(const mavros_msgs::State::ConstPtr&);
  void Quadrotor_Callback(const geometry_msgs::TransformStamped::ConstPtr&);
  void Hexarotor_CallBack(const geometry_msgs::TransformStamped::ConstPtr&);
  void RC_In_CallBack(const mavros_msgs::RCIn::ConstPtr&);
  void Dist_Publish();

 private:
  ros::Subscriber State_sub_;
  ros::Subscriber Quadrotor_sub_;
  ros::Subscriber Hexarotor_sub_;
  ros::Subscriber RC_In_sub_;
  ros::Publisher Disturbance_pub_;
  bool is_arm_;
  uint8_t Docking_mode_count_;
  geometry_msgs::TransformStamped Quadrotor_data_;
  geometry_msgs::TransformStamped Hexarotor_data_;
  geometry_msgs::Vector3 Disturbance_;
  float p00_;
  float p10_;
  float p01_;
  float p20_;
  float p11_;
  float p02_;
  float p30_;
  float p21_;
  float p12_;
};
