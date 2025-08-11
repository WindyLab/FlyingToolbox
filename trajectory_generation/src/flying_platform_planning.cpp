#include "flying_platform_planning.h"


flying_platform_planning::flying_platform_planning( )
{
    set_work_space(Eigen::MatrixXd::Identity(3,3),
                                     Eigen::Vector3d(0.08,0.08,-0.15),
                                     Eigen::Vector3d(-0.08,-0.08,-0.30),
                                     0.5*(Eigen::Vector3d(0.08,0.08,-0.15) + Eigen::Vector3d(-0.08,-0.08,-0.30)));
}

void flying_platform_planning::get_manipulator_offset(double yaw_offset, Eigen::Vector3d delta_offset)
{
    yaw_offset_ = yaw_offset;
    delta_offset_ = delta_offset;
    rotation_delta_b_ << -sin(yaw_offset_), cos(yaw_offset_), 0, cos(yaw_offset_), sin(yaw_offset_), 0, 0, 0, -1;
}

void flying_platform_planning::get_pick_task_param(double time_reach, double time_contact, double time_hold, 
                                                        double vel_reach,double vel_cont, double rate_cont_reach)
{
    time_reach_ = time_reach;
    time_contact_ = time_contact;
    time_hold_ = time_hold;
    vel_reach_ = vel_reach;
    vel_cont_ = vel_cont;
    rate_cont_reach_ = rate_cont_reach;
}


void flying_platform_planning::trajectory_out(ros::Time timeNow, path_point& traj_sp)
{
    trajectory_solver_.trajectory_out(timeNow, traj_sp);
    // std::cout << i<<std::endl;
}

void flying_platform_planning::get_objectives(vector<pick_object> obj)
{
    obj_.clear();
    obj_.assign(obj.begin(),obj.end());
}

void flying_platform_planning::get_constrants(vector<constraint> cons, path_point start_pt, path_point end_pt)
{
    trajectory_solver_.get_path_constraint(cons, start_pt, end_pt);
}

void flying_platform_planning::generate_reachable_space()
{
    Eigen::Vector3d dst_end_quad = rotation_delta_b_ * workspace_.center + delta_offset_;
    int n_obj = obj_.size();
    for (size_t i = 0; i < n_obj; i++)
    {
        constraint temp;
        temp.s_scale = time_reach_;
        temp.upper_vel = vel_reach_*Eigen::Vector3d(1,1,1);
        temp.lower_vel  = vel_reach_*Eigen::Vector3d(-1,-1,-1);
        temp.upper_acc << 1,1,1;
        temp.lower_acc << -1,-1,-1;
        temp.A_pos.setIdentity(3,3);
        temp.b_u_pos.resize(3);
        temp.b_u_pos = obj_[i].pos - dst_end_quad + workspace_.b_u_w;
        temp.b_l_pos.resize(3);
        temp.b_l_pos  = obj_[i].pos - dst_end_quad + workspace_.b_l_w;
        reachable_space_.push_back(temp);
        constraint temp_cont;
        temp_cont.s_scale = time_contact_;
        temp_cont.upper_vel << vel_reach_*Eigen::Vector3d(1,1,1);
        temp_cont.lower_vel << vel_reach_*Eigen::Vector3d(-1,-1,-1);
        temp_cont.upper_acc << 1,1,1;
        temp_cont.lower_acc << -1,-1,-1;
        temp_cont.A_pos.setIdentity(3,3);
        temp_cont.b_u_pos.resize(3);
        temp_cont.b_u_pos = obj_[i].pos - dst_end_quad + rate_cont_reach_ * workspace_.b_u_w;
        temp_cont.b_l_pos.resize(3);
        temp_cont.b_l_pos  = obj_[i].pos - dst_end_quad + rate_cont_reach_ * workspace_.b_l_w;
        contact_begin_space_.push_back(temp_cont);
    }
}


void flying_platform_planning::set_work_space(Eigen::MatrixXd A_w, Eigen::VectorXd b_u_w, 
                                                                                                    Eigen::VectorXd b_l_w, Eigen::Vector3d center)
{
    workspace_.A_w = A_w;
    workspace_.b_u_w = b_u_w;
    workspace_.b_l_w = b_l_w;
    workspace_.center = center;
}

bool flying_platform_planning::traj_out_call(trajectory_generation::traj_out_msgRequest& request,trajectory_generation::traj_out_msgResponse& response){
    // time align
    if (!task_begin_flag)
    {
        task_begin_flag = true;
        trajectory_solver_.time_align();
    }
    
    path_point traj_sp;
    trajectory_solver_.trajectory_out(request.times, traj_sp);
    response.x    = traj_sp.pos[0];
    response.y    = traj_sp.pos[1];
    response.z    = traj_sp.pos[2];

    response.dx   = traj_sp.vel[0];
    response.dy   = traj_sp.vel[1];
    response.dz   = traj_sp.vel[2];

    response.ddx  = traj_sp.acc[0]; 
    response.ddy  = traj_sp.acc[1];
    response.ddz  = traj_sp.acc[2];
    response.flag_begin = true;
    ROS_INFO("%f\t%f\t%f\n",response.x,response.y,response.z);
    return 1;
}
void flying_platform_planning::run()
{
    trajectory_solver_.trajectory_solver_QP();
}

void flying_platform_planning::bezier_result_out(vector<bezier>& bezier_3d)
{
    trajectory_solver_.bezier_result_out(bezier_3d);
}
