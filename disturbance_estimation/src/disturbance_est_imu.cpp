#include "disturbance_est_imu.h"
#define Hexarotor 6

disturbance_est_imu::disturbance_est_imu(ros::NodeHandle &n_) {

  State_sub_ = n_.subscribe<mavros_msgs::State>(
      "/mavros/state", 10, &disturbance_est_imu::State_CallBack, this);
  Att_sub_ = n_.subscribe<sensor_msgs::Imu>(
      "/mavros/imu/data", 10, &disturbance_est_imu::Att_CallBack, this);
  Quadrotor_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Turing/Turing", 10, &disturbance_est_imu::Quadrotor_Callback, this);
  Hexarotor_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Galileo/Galileo", 10, &disturbance_est_imu::Hexarotor_CallBack, this);
  IMU_acc_sub_ = n_.subscribe<geometry_msgs::Vector3>(
      "/wit/imu_acc_filtered", 10, &disturbance_est_imu::IMU_Acc_CallBack, this);
  Motor_sub_ = n_.subscribe<disturbance_estimation::motor_state>(
      "/motor_state", 10, &disturbance_est_imu::Motor_state_CallBack, this);
  RC_In_sub_ = n_.subscribe<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, &disturbance_est_imu::RC_In_CallBack, this);
  attitude_sub = n_.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10,  &disturbance_est_imu::att_obtain,this);
  Disturbance_force_pub_ = n_.advertise<geometry_msgs::Vector3>(
      "/disturbance_truth/force", 10);
      
  ros::param::param("~p1", para_thrust_[0], -6.939e-7);
  ros::param::param("~p2", para_thrust_[1], 3.127e-4);
  ros::param::param("~p3", para_thrust_[2], -0.2451);
  ros::param::param("~p4", para_thrust_[3], 1.1122);
  ros::param::param("~m", mass_aerbox_,6.837);
  G_[0] = 0.0f;
  G_[1] = 0.0f;
  G_[2] = mass_aerbox_ * 9.81;

  is_arm_ = false;
  Docking_mode_count_ = 0;
  is_calibration_ = false;
  IMU_Acc_Bias_ = MatrixXd::Zero(3,1);  

  char *buffer;
  if ((buffer = getcwd(NULL, 0)) == NULL) {
    printf("getcwd error\n");
  } else {
    printf("%s\n", buffer);
    free(buffer);
  }

}
disturbance_est_imu::~disturbance_est_imu() {}

/**
 * @brief switch the Quaterniond to euler angle
 *
 * @param Quaterniond input Quaterniond data [w,x,y,z]
 *
 * @return Vector3d Euler angle [yaw, pitch, roll ]
 */
Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q) {
  Eigen::Vector3d angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles(2) = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    angles(1) =
        std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles(1) = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles(0) = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}
/**
 * @brief check whether the vehicle is arm
 *
 * @param State_data mavros_msgs::State
 * 
 */
void disturbance_est_imu::State_CallBack(
    const mavros_msgs::State::ConstPtr &State_data) {
  if (State_data.get()->armed)
    is_arm_ = true;
  else
    is_arm_ = false;
}
/**
 * @brief IMU state callback function
 *
 * @param Imu_data sensor_msgs::Imu 
 * 
 */
void disturbance_est_imu::Att_CallBack(
    const sensor_msgs::Imu::ConstPtr &Imu_data) {
  float x = Imu_data.get()->orientation.x;
  float y = Imu_data.get()->orientation.y;
  float z = Imu_data.get()->orientation.z;
  float w = Imu_data.get()->orientation.w;
  Re3_[0] = 2.0 * (x * z + y * w);
  Re3_[1] = 2.0 * (y * z - x * w);
}
/**
 * @brief Quadrotor Position callback function
 *
 * @param Quad_data geometry_msgs::TransformStamped
 * 
 */
void disturbance_est_imu::Quadrotor_Callback(const geometry_msgs::TransformStamped::ConstPtr &Quad_data) {
  Quadrotor_data_.header.stamp = Quad_data.get()->header.stamp;
  Quadrotor_data_.transform.translation.x = Quad_data.get()->transform.translation.x;
  Quadrotor_data_.transform.translation.y = Quad_data.get()->transform.translation.y;
  Quadrotor_data_.transform.translation.z = Quad_data.get()->transform.translation.z;
}
/**
 * @brief Hexarotor Position and Attitude callback function
 *
 * @param Hexa_data geometry_msgs::TransformStamped
 * 
 */
void disturbance_est_imu::Hexarotor_CallBack(
    const geometry_msgs::TransformStamped::ConstPtr &Hexa_data) {
  Hexarotor_data_.header.stamp = Hexa_data.get()->header.stamp;
  Hexarotor_data_.transform.translation.x = Hexa_data.get()->transform.translation.x;
  Hexarotor_data_.transform.translation.y = Hexa_data.get()->transform.translation.y;
  Hexarotor_data_.transform.translation.z = Hexa_data.get()->transform.translation.z;
  float x = Hexa_data.get()->transform.rotation.x;
  float y = Hexa_data.get()->transform.rotation.y;
  float z = Hexa_data.get()->transform.rotation.z;
  float w = Hexa_data.get()->transform.rotation.w;
  Re3_[2] = -x * x - y * y + z * z + w * w;
}

void  disturbance_est_imu::att_obtain(const sensor_msgs::Imu::ConstPtr& msg)
{
  Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  //Transform the Quaternion to euler Angles
  Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
  // flightStateData.phi    = euler_fcu[0];
  // flightStateData.theta  = - euler_fcu[1];
  lower_uav_psi_    = - euler_fcu[2] ;
}


/**
 * @brief Filtered Acceleration from IMU module callback function
 *
 * @param IMU_Acc_data geometry_msgs::Vector3
 * 
 */
void disturbance_est_imu::IMU_Acc_CallBack(
    const geometry_msgs::Vector3::ConstPtr &IMU_Acc_data) {
  IMU_Acc_(0) = IMU_Acc_data.get()->x;
  IMU_Acc_(1) = IMU_Acc_data.get()->y;
  IMU_Acc_(2) = IMU_Acc_data.get()->z;
}
/**
 * @brief moter state callback function
 *
 * @param Motor_State_data disturbance_estimation::motor_state
 * 
 */
void disturbance_est_imu::Motor_state_CallBack(
    const disturbance_estimation::motor_state::ConstPtr &Motor_State_data) {
  motor_state_.stamp = Motor_State_data.get()->stamp;
  for (int i = 0; i < 8; i++) {
    motor_state_.packetNumber[i] = Motor_State_data.get()->packetNumber[i];
    motor_state_.rcThrottle[i] = Motor_State_data.get()->rcThrottle[i];
    motor_state_.actualThrottle[i] = Motor_State_data.get()->actualThrottle[i];
    motor_state_.electricalSpeed[i] = Motor_State_data.get()->electricalSpeed[i];
    motor_state_.busVoltage[i] = Motor_State_data.get()->busVoltage[i];
    motor_state_.busCurrent[i] = Motor_State_data.get()->busCurrent[i];
    motor_state_.phaseCurrent[i] = Motor_State_data.get()->phaseCurrent[i];
    motor_state_.mosTemperature[i] = Motor_State_data.get()->mosTemperature[i];
    motor_state_.capacitorTemperature[i] = Motor_State_data.get()->capacitorTemperature[i];
    motor_state_.statusCode[i] = Motor_State_data.get()->statusCode[i];
  }
}
/**
 * @brief Disturbance access switch 
 *
 * @param RC_In_data mavros_msgs::RCIn
 * 
 */
void disturbance_est_imu::RC_In_CallBack(
    const mavros_msgs::RCIn::ConstPtr &RC_In_data) {
  if (RC_In_data.get()->channels[10] > 1500) {
    if (Docking_mode_count_ < 30) Docking_mode_count_++;
  } else {
    if (Docking_mode_count_ > 0) Docking_mode_count_--;
  }
  if (Docking_mode_count_ == 0) is_calibration_ = false;
}



void disturbance_est_imu::Calculate_Thrust() {
  T_ = 0;
  for (int i = 0; i < Hexarotor; i++) {
    T_ -= para_thrust_[0] * motor_state_.electricalSpeed[i] *
              motor_state_.electricalSpeed[i] +
          para_thrust_[1] * motor_state_.electricalSpeed[i] + para_thrust_[2] + para_thrust_[3];
  }

}

void disturbance_est_imu::Calculate_Disturbance() {
  Calculate_Thrust();
  F_ = -Re3_ * T_ + G_;
  IMU_Acc_Bias_ = Eigen::Vector3d(0, 0, 0);
  Disturbance_ = (IMU_Acc_ - IMU_Acc_Bias_) * mass_aerbox_ - F_;
}


void disturbance_est_imu::saveGroundThruth(){
  Calculate_Disturbance();
  geometry_msgs::Vector3 estimate_force;
  // LOGFMTD("%s %f", "Disturbance_x", Disturbance_(0));
  // LOGFMTD("%s %f", "Disturbance_y", Disturbance_(1));
  // LOGFMTD("%s %f", "Disturbance_z", Disturbance_(2));
  estimate_force.x = Disturbance_(0);
  estimate_force.y = Disturbance_(1);
  estimate_force.z = Disturbance_(2);
  Disturbance_force_pub_.publish(estimate_force);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "disturbance_est_imu_node");
  ros::NodeHandle nh;

  disturbance_est_imu disturbance_est_imu_obj(nh);

  // disturbance_est_imu_obj.test();
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    disturbance_est_imu_obj.saveGroundThruth();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
