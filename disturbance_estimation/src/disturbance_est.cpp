#include "disturbance_est.h"
#include "nn_predict.h"
#include "nn_predict_terminate.h"

disturbance_est::disturbance_est(ros::NodeHandle &n_) {

  State_sub_ = n_.subscribe<mavros_msgs::State>(
      "/mavros/state", 10, &disturbance_est::State_CallBack, this);
  Quadrotor_Edison_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Edison/Edison", 10, &disturbance_est::Edison_Callback, this);
  Quadrotor_Turing_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Turing/Turing", 10, &disturbance_est::Turing_Callback, this);
  Hexarotor_sub_ = n_.subscribe<geometry_msgs::TransformStamped>(
      "/vicon/Galileo/Galileo", 10, &disturbance_est::Hexarotor_CallBack, this);
  RC_In_sub_ = n_.subscribe<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, &disturbance_est::RC_In_CallBack, this);
  Disturbance_torque_pub_ = n_.advertise<geometry_msgs::Vector3>(
      "/mavros/disturbance_estimate/torque", 10);
  Disturbance_force_pub_ = n_.advertise<geometry_msgs::Vector3>(
      "/mavros/disturbance_estimate/force", 10);
  attitude_sub = n_.subscribe<sensor_msgs::Imu>(
      "mavros/imu/data", 10,  &disturbance_est::att_obtain,this);

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
void disturbance_est::Edison_Callback(const geometry_msgs::TransformStamped::ConstPtr &Quad_data) {
  Quadrotor_Edison_data_.header.stamp = Quad_data.get()->header.stamp;
  Quadrotor_Edison_data_.transform.translation.x = Quad_data.get()->transform.translation.x;
  Quadrotor_Edison_data_.transform.translation.y = Quad_data.get()->transform.translation.y;
  Quadrotor_Edison_data_.transform.translation.z = Quad_data.get()->transform.translation.z;
}
/**
 * @brief Quadrotor Position callback function
 *
 * @param Quad_data geometry_msgs::TransformStamped
 * 
 */
void disturbance_est::Turing_Callback(const geometry_msgs::TransformStamped::ConstPtr &Quad_data) {
  Quadrotor_Turing_data_.header.stamp = Quad_data.get()->header.stamp;
  Quadrotor_Turing_data_.transform.translation.x = Quad_data.get()->transform.translation.x;
  Quadrotor_Turing_data_.transform.translation.y = Quad_data.get()->transform.translation.y;
  Quadrotor_Turing_data_.transform.translation.z = Quad_data.get()->transform.translation.z;
}

/**
 * @brief Hexarotor Position and Attitude callback function
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

void  disturbance_est::att_obtain(const sensor_msgs::Imu::ConstPtr& msg)
{
  Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  //Transform the Quaternion to euler Angles
  Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
  // flightStateData.phi    = euler_fcu[0];
  // flightStateData.theta  = - euler_fcu[1];
  lower_uav_psi_    = - euler_fcu[2] ;
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

void disturbance_est::airflow_model(double r, double dz,double& vel){
  double para[5] = {13.2740,1.4545,0.2015,0.0694,0.0850};
  double v_max = para[0] - para[1]*dz;
  vel = v_max*exp(-0.5*((r-para[2])/(para[3]+para[4]*dz))*((r-para[2])/(para[3]+para[4]*dz)));
}


void disturbance_est::PINN_estimate(Eigen::Vector3d upper_uav_pos,Eigen::Vector3d lower_uav_pos,geometry_msgs::Vector3& estimate_force,geometry_msgs::Vector3& estimate_torque){
  float delta_x = upper_uav_pos[0] - lower_uav_pos[0];
  float delta_y = upper_uav_pos[1] - lower_uav_pos[1];
  float delta_z = upper_uav_pos[2] - lower_uav_pos[2];
  vector <Eigen::Vector3d> pos_rotor_body;
  pos_rotor_body.resize(6);
  double wheelbase = 0.502;
  for (size_t i = 0; i < 6; i++)
  {
    pos_rotor_body[i]=Eigen::Vector3d(cos((i)*pi/3.0)*wheelbase, sin(i*pi/3.0)*wheelbase,0);
  }
  vector <Eigen::Vector3d> pos_rotor;
  pos_rotor.resize(6);
  Eigen::Matrix3d R_psi;
  R_psi << cos(lower_uav_psi_),-sin(lower_uav_psi_),0,
            sin(lower_uav_psi_),cos(lower_uav_psi_),0,
            0,0,1;
  for (size_t i = 0; i < 6; i++)
  {
    pos_rotor[i] = lower_uav_pos + R_psi*pos_rotor_body[i];
  }

  vector <Eigen::Vector3d> delta_pos_rotor;
  delta_pos_rotor.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    delta_pos_rotor[i] = upper_uav_pos - pos_rotor[i];
  }

  
  vector<double> v_rotor;
  v_rotor.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    double r = sqrt(delta_pos_rotor[i](0)*delta_pos_rotor[i](0) + delta_pos_rotor[i](1)*delta_pos_rotor[i](1));
    double dz = abs(delta_pos_rotor[i](2));
    airflow_model( r,  dz, v_rotor[i]);
  }
  
 
  float r_uav = sqrt(delta_x * delta_x + delta_y * delta_y);
  float dz_uav = abs(delta_z);

  LOGFMTD("%s %f", "r_uav", r_uav);
  LOGFMTD("%s %f", "dz_uav", dz_uav);

  // check motor armed and motor state update
  if (is_arm_ && Docking_mode_count_ > 0 && UAV_position_update)
  {
    double input_nn[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    for (size_t i = 0; i < 6; i++)
    {
      input_nn[i] = v_rotor[i];
    }
    input_nn[6] = r_uav;
    input_nn[7] = dz_uav;
    float out;
    // Initialize function 'nn_predict' input arguments.
    // Initialize function input argument 'in'.
    // Call the entry-point 'nn_predict'.
    out = nn_predict(input_nn);
    // std::cout << out << std::endl;
    estimate_force.x =0;
    estimate_force.y =0;
    estimate_force.z =out;

    double sum_v_rotor = 0;
    for (size_t i = 0; i < 6; i++)
    {
      sum_v_rotor = sum_v_rotor + v_rotor[i];
    }
    
    double cd = out/sum_v_rotor;

    Eigen::Vector3d n_vec ={0,0,1};
    Eigen::Vector3d r_cross={0,0,0};
    for (size_t i = 0; i < 6; i++)
    {
      r_cross = r_cross + pos_rotor_body[i].cross(n_vec) * cd * v_rotor[i] /6.0;
    }
    if (r_uav > 0.05 && dz_uav > 0.65)
    {
      estimate_torque.x = -r_cross[0];
      estimate_torque.y = -r_cross[1] * 2.0;
      estimate_torque.z = r_cross[2];
    }
    else
    {
      estimate_torque.x = 0.0;
      estimate_torque.y = 0.0;
      estimate_torque.z = 0.0;
    }  

    if (!(r_uav < 1.5 && delta_z < 0))
    {
      estimate_force.x =0.0;
      estimate_force.y =0.0;
      estimate_force.z =0.0;
    }
  }
  else
  {
    estimate_force.x =0.0;
    estimate_force.y =0.0;
    estimate_force.z =0.0;
    estimate_torque.x = 0.0;
    estimate_torque.y = 0.0;
    estimate_torque.z = 0.0;
  }
}

bool disturbance_est::vicon_data_is_avilable(geometry_msgs::TransformStamped in_data)
{
  bool data_is_null;
  if ((in_data.transform.translation.x < 4.0) && (in_data.transform.translation.x > -4.0)&&
      (in_data.transform.translation.y < 4.0) && (in_data.transform.translation.y > -4.0) &&
      (in_data.transform.translation.z < 5.0) && (in_data.transform.translation.z > -1.0))
  {
    data_is_null = false;
  }
  else
  {
    data_is_null = true;
  }

  if ((ros::Time::now() - in_data.header.stamp).toSec() < 0.2 && (!data_is_null))
  {
    return true;
  }
  else 
  {
    return false;
  }
}

void disturbance_est::sover_estimate() {
  Eigen::Vector3d lower_uav_pos(Hexarotor_data_.transform.translation.x,-Hexarotor_data_.transform.translation.y,-Hexarotor_data_.transform.translation.z);
  Eigen::Vector3d upper_uav_pos;
  //Quadrotor_Turing_data_ 
  //Quadrotor_Edison_data_
  // Four cases
  // Case 1: Both
  // Case 2: Turing, but not Edison
  // Case 3: Edison, but not Turing
  // Case 4: Neither
  // Cannot have NULL
  Eigen::Vector3d Turing_pos(Quadrotor_Turing_data_.transform.translation.x,-Quadrotor_Turing_data_.transform.translation.y,-Quadrotor_Turing_data_.transform.translation.z) ;
  Eigen::Vector3d Edison_pos(Quadrotor_Edison_data_.transform.translation.x,-Quadrotor_Edison_data_.transform.translation.y,-Quadrotor_Edison_data_.transform.translation.z) ;
  if (vicon_data_is_avilable(Quadrotor_Turing_data_) && vicon_data_is_avilable(Quadrotor_Edison_data_))
  {
    double dis_turing = sqrt(   (Turing_pos(0) - lower_uav_pos(0))* (Turing_pos(0) - lower_uav_pos(0))  
                              + (Turing_pos(1) - lower_uav_pos(1))* (Turing_pos(1) - lower_uav_pos(1)));
    double dis_edison = sqrt(   (Edison_pos(0) - lower_uav_pos(0))* (Edison_pos(0) - lower_uav_pos(0))  
                              + (Edison_pos(1) - lower_uav_pos(1))* (Edison_pos(1) - lower_uav_pos(1)));
    if (dis_turing < dis_edison)
    {
      upper_uav_pos = Turing_pos; 
      LOGFMTD("%s %d", "Position from", 0);
      ROS_INFO("Position from Turing!");
    }
    else
    {
      upper_uav_pos = Edison_pos;
      LOGFMTD("%s %d", "Position from", 1);
      ROS_INFO("Position from Edison!");
    }
    UAV_position_update = true;
    
  }
  else if (vicon_data_is_avilable(Quadrotor_Turing_data_) && (!vicon_data_is_avilable(Quadrotor_Edison_data_)))
  {
    upper_uav_pos = Turing_pos;
    UAV_position_update = true;
    ROS_INFO("Position from Turing!");
  }
  else if ((!vicon_data_is_avilable(Quadrotor_Turing_data_)) && vicon_data_is_avilable(Quadrotor_Edison_data_))
  {
    upper_uav_pos = Edison_pos;
    UAV_position_update = true;
    ROS_INFO("Position from Edison!");
  }
  else
  {
    UAV_position_update = false;
    upper_uav_pos << lower_uav_pos(0) + 2,lower_uav_pos(1) + 2,lower_uav_pos(2);
    ROS_INFO("No position from vicon!");
  }
  
  geometry_msgs::Vector3 estimate_force;
  geometry_msgs::Vector3 estimate_torque;
  PINN_estimate(upper_uav_pos,lower_uav_pos,estimate_force,estimate_torque);
  estimate_torque.x   = estimate_torque.x * 3;
  // estimate_torque.x   = 0.0;
  // estimate_torque.y   = 0.0;
  // estimate_torque.z   = 0.0;
  estimate_force.z = estimate_force.z * 0.65;
  // estimate_force_z_last = 0.8 * estimate_force_z_last + 0.2 * estimate_force.z;
  // estimate_force.z = estimate_force_z_last;
  LOGFMTD("%s %f", "upper_uav_pos_x", upper_uav_pos[0]);
  LOGFMTD("%s %f", "upper_uav_pos_y", upper_uav_pos[1]);
  LOGFMTD("%s %f", "upper_uav_pos_z", upper_uav_pos[2]);
  LOGFMTD("%s %f", "lower_uav_pos_x", lower_uav_pos[0]);
  LOGFMTD("%s %f", "lower_uav_pos_y", lower_uav_pos[1]);
  LOGFMTD("%s %f", "lower_uav_pos_z", lower_uav_pos[2]);
  Disturbance_force_pub_.publish(estimate_force);
  LOGFMTD("%s %f", "estimate_force_x", estimate_force.x);
  LOGFMTD("%s %f", "estimate_force_y", estimate_force.y);
  LOGFMTD("%s %f", "estimate_force_z", estimate_force.z);
  Disturbance_torque_pub_.publish(estimate_torque);
  LOGFMTD("%s %f", "estimate_torque_x", estimate_torque.x);
  LOGFMTD("%s %f", "estimate_torque_y", estimate_torque.y);
  LOGFMTD("%s %f", "estimate_torque_z", estimate_torque.z);
  // LOGFMTD("%s %f", "lower_uav_psi_", lower_uav_psi_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "disturbance_est_node");
  ros::NodeHandle nh;

  disturbance_est disturbance_est_obj(nh);
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    disturbance_est_obj.sover_estimate();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
