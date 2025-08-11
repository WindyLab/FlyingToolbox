#include "disturbance_est.h"
#define Quadrotor 4
#define Hexarotor 6

using namespace std;

/**
 * @brief check whether the vehicle is arm
 *
 * @param State_data mavros_msgs::State
 * 
 */
void disturbance_est::State_CallBack(
    const mavros_msgs::State::ConstPtr &State_data) {
  if (State_data.get()->armed)
    is_arm_ = true;
  else
    is_arm_ = false;
}

/**
 * @brief Quadrotor Position callback function
 *
 * @param Quad_data geometry_msgs::TransformStamped
 * 
 */
void disturbance_est::Quadrotor_Callback(const geometry_msgs::TransformStamped::ConstPtr &Quad_data) {
  Quadrotor_data_.header.stamp = Quad_data.get()->header.stamp;
  Quadrotor_data_.transform.translation.x = Quad_data.get()->transform.translation.x;
  Quadrotor_data_.transform.translation.y = Quad_data.get()->transform.translation.y;
  Quadrotor_data_.transform.translation.z = Quad_data.get()->transform.translation.z;
}
/**
 * @brief Hexarotor Position callback function
 *
 * @param Hexa_data geometry_msgs::TransformStamped
 * 
 */
void disturbance_est::Hexarotor_CallBack(
    const geometry_msgs::TransformStamped::ConstPtr &Hexa_data) {
  Hexarotor_data_.header.stamp = Hexa_data.get()->header.stamp;
  Hexarotor_data_.transform.translation.x = Hexa_data.get()->transform.translation.x;
  Hexarotor_data_.transform.translation.y = Hexa_data.get()->transform.translation.y;
  Hexarotor_data_.transform.translation.z = Hexa_data.get()->transform.translation.z;
}
/**
 * @brief Disturbance access switch 
 *
 * @param RC_In_data mavros_msgs::RCIn
 * 
 */
void disturbance_est::RC_In_CallBack(
    const mavros_msgs::RCIn::ConstPtr &RC_In_data) {
  if (RC_In_data.get()->channels[10] > 1500) {
    if (Docking_mode_count_ < 30) Docking_mode_count_++;
  } else {
    if (Docking_mode_count_ > 0) Docking_mode_count_--;
  }
}

disturbance_est::disturbance_est(ros::NodeHandle &n_) {
    p00_ =       8.221;
    p10_ =       -28.3;
    p01_ =       31.66;
    p20_ =       13.91;
    p11_ =      -10.39;
    p02_ =      -14.42;
    p30_ =      -1.779;
    p21_ =     -0.2105;
    p12_ =        5.05;

  State_sub_ = n_.subscribe<mavros_msgs::State>(
      "/mavros/state", 10, &disturbance_est::State_CallBack, this);
  Quadrotor_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Edison/Edison", 10, &disturbance_est::Quadrotor_Callback, this);
  Hexarotor_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Galileo/Galileo", 10, &disturbance_est::Hexarotor_CallBack, this);
  RC_In_sub_ = n_.subscribe<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, &disturbance_est::RC_In_CallBack, this);
  Disturbance_pub_ = n_.advertise<geometry_msgs::Vector3>(
      "/mavros/disturbance_est/dist_est", 10);

  is_arm_ = false;
  Docking_mode_count_ = 0;

  char *buffer;
  if ((buffer = getcwd(NULL, 0)) == NULL) {
    printf("getcwd error\n");
  } else {
    printf("%s\n", buffer);
    free(buffer);
  }
  // start log4z
  zsummer::log4z::ILog4zManager::getRef().start();
}
disturbance_est::~disturbance_est() {}

void disturbance_est::Dist_Publish() {
  float delta_x = (Quadrotor_data_.transform.translation.x -
                   Hexarotor_data_.transform.translation.x);
  float delta_y = (Quadrotor_data_.transform.translation.y -
                   Hexarotor_data_.transform.translation.y);
  float delta_h = (Quadrotor_data_.transform.translation.z -
                   Hexarotor_data_.transform.translation.z);

  float delta_xy = sqrt(delta_x * delta_x + delta_y * delta_y);

  bool UAV_position_update;
  if((ros::Time::now() - Quadrotor_data_.header.stamp).toSec() < 1.0 && 
      (ros::Time::now() - Hexarotor_data_.header.stamp).toSec() < 1.0){
          UAV_position_update = true;
      }
  else{
    UAV_position_update = false;
    ROS_ERROR("Get position from vicon Error!");
  }
  // check motor armed and motor state update
  if (is_arm_ && Docking_mode_count_ > 0 && delta_xy < 2.0 && UAV_position_update) {

    Disturbance_.x = 0.0;
    Disturbance_.y = 0.0;
    Disturbance_.z =
        p00_ + p10_ * delta_xy + p01_ * delta_h + p20_ * delta_xy * delta_xy +
        p11_ * delta_xy * delta_h + p02_ * delta_h * delta_h + p30_ * delta_xy * delta_xy * delta_xy +
        p21_ * delta_xy * delta_xy * delta_h + p12_ * delta_xy * delta_h * delta_h;
    Disturbance_.z = 0.6 * Disturbance_.z;
    Disturbance_.z = Disturbance_.z > 25 ? 25 : Disturbance_.z;
    Disturbance_.z = Disturbance_.z < 0 ? 0 : Disturbance_.z;
    Disturbance_.z = Disturbance_.z * Docking_mode_count_ / 30.0;

  } else {
    Disturbance_.x = 0;
    Disturbance_.y = 0;
    Disturbance_.z = 0;
  }

  LOGFMTD("%s %d", "Docking_mode_count_", Docking_mode_count_);
  LOGFMTD("%s %f", "Disturbance_", Disturbance_.z);
  Disturbance_pub_.publish(Disturbance_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "disturbance_est_node");
  ros::NodeHandle nh;

  disturbance_est disturbance_estimate(nh);
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    disturbance_estimate.Dist_Publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
