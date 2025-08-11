# ifndef FLY_PLAT_PLAN
# define FLY_PLAT_PLAN

#include "trajectory_solver.h"
#include "type_define.h"



class flying_platform_planning
{
public:
  flying_platform_planning();
  ~flying_platform_planning(){};
  void get_manipulator_offset(double yaw_offset, Eigen::Vector3d delta_offset);
  void get_pick_task_param(double time_reach, double time_contact, double time_hold, 
                                                        double vel_reach,double vel_cont, double rate_cont_reach);
  // void move_stage_planning();
  void get_objectives(vector<pick_object> obj);
  void get_constrants(vector<constraint> cons, path_point start_pt, path_point end_pt);
  void run();
  void trajectory_out(ros::Time timeNow, path_point& traj_sp);
  bool traj_out_call(trajectory_generation::traj_out_msgRequest& request,trajectory_generation::traj_out_msgResponse& response);
  void bezier_result_out(vector<bezier>& bezier_3d);


private:
  trajectory_solver trajectory_solver_;


  void set_work_space(Eigen::MatrixXd A_w, Eigen::VectorXd b_u_w, Eigen::VectorXd b_l_w, Eigen::Vector3d center);
  void generate_reachable_space();
  


  vector<pick_object> obj_;
  vector<constraint> reachable_space_;
  vector<constraint> contact_begin_space_;
  workspace workspace_;
  Eigen::Matrix3d rotation_delta_b_;
  Eigen::Vector3d delta_offset_;
  double yaw_offset_;

  double time_reach_; // from launch file
  double begin_time_; // need calculate
  double time_contact_; // from launch file
  double time_hold_; // from launch file
  double vel_reach_; // from launch file
  double vel_cont_; // from launch file
  double rate_cont_reach_; // from launch file

  // flags                                                                                                                                                                
  bool task_begin_flag = false;
};

# endif