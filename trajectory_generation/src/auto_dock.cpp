#include"auto_dock.h"

auto_dock::auto_dock(/* args */)
{
    traj_result_server = nh_.advertiseService("trajectory_result", &auto_dock::traj_out_call,this);
    tool_chatter_pub = nh_.advertise<std_msgs::String>("chatter_tool", 1000);
    switch2docking = nh_.advertise<std_msgs::Int16>("switch_to_docking", 1000);
    switch2moving = nh_.advertise<std_msgs::Bool>("switch_to_moving", 1000);
    switch2flyingup = nh_.advertise<std_msgs::Bool>("switch_to_flyingup", 1000);
    offset_end_pub = nh_.advertise<std_msgs::Float32MultiArray>("offest_end_task", 1000);
    pos_arrive_sub = nh_.subscribe("pos_is_arrive",10, &auto_dock::pos_arrive_callback,this);
    int bezier_order_flying,bezier_order_end,optimal_order_flying,optimal_order_end;
    double tan_gamma_cone, time_to_cone;
    nh_.param<int>("/auto_dock/bezier_order_flying",bezier_order_flying,8);
    nh_.param<int>("/auto_dock/optimal_order_flying",optimal_order_flying,3);

    XmlRpc::XmlRpcValue num_path,task_data;
    nh_.getParam("/auto_pick/num_path",num_path);
    nh_.getParam("/auto_pick/task_data",task_data);
    get_path_cooridor(num_path, task_data);
    std::cout << "Corridor  have been loaded for quadcopter." << std::endl;
    flying_plan_.clear();
    int num_path_i  = num_path;
    flying_plan_.resize(num_path_i);
    for (size_t i = 0; i < num_path_i; i++)
    {
        flying_plan_[i].set_bezier_solver( bezier_order_flying, optimal_order_flying);
    }
}


void auto_dock::pos_arrive_callback(const std_msgs::Bool::ConstPtr &msg )
{
    flag_target_position_pos_ = true;
    time_hold_begin = ros::Time::now();
    ROS_INFO("Target is arrived!");
}

bool auto_dock::traj_out_call(trajectory_generation::traj_out_msgRequest& request,trajectory_generation::traj_out_msgResponse& response)
{
    // time align
    if (!task_begin_flag_ && task_index_ <flying_plan_.size())
    {
        task_begin_flag_ = true;
        flying_plan_[task_index_].time_align();
        ROS_INFO("Trajectory of the quadcopter is sending!");
    }
    path_point traj_sp;
    
    int task_index_real = (task_index_ > (flying_plan_.size()-1))? (flying_plan_.size()-1):task_index_;
    bool flag_tmp;
    if (flying_plan_[task_index_real].trajectory_out(request.times, traj_sp, flag_tmp))
    {
        response.x    = traj_sp.pos[0];
        response.y    = traj_sp.pos[1];
        response.z    = traj_sp.pos[2];

        response.dx   = traj_sp.vel[0];
        response.dy   = traj_sp.vel[1];
        response.dz   = traj_sp.vel[2];

        response.ddx  = traj_sp.acc[0]; 
        response.ddy  = traj_sp.acc[1];
        response.ddz  = traj_sp.acc[2];
        response.psi  = psi_angle_task_[task_index_real];
        response.flag_begin = true;
        flag_begin_dock_ = flag_tmp;
    }
    else
    {
        response.x    = start_pt_[task_index_real].pos[0];
        response.y    = start_pt_[task_index_real].pos[1];
        response.z    = start_pt_[task_index_real].pos[2];

        response.dx   = start_pt_[task_index_real].vel[0];
        response.dy   = start_pt_[task_index_real].vel[1];
        response.dz   = start_pt_[task_index_real].vel[2];

        response.ddx  = start_pt_[task_index_real].acc[0]; 
        response.ddy  = start_pt_[task_index_real].acc[1];
        response.ddz  = start_pt_[task_index_real].acc[2];
        response.psi  = psi_angle_task_[task_index_real];
        response.flag_begin = true;
    }
        
    return 1;
}

void auto_dock::show_corridor()
{
    int num = corridor_.size();
    for (size_t i = 0; i < num; i++)
    {
        int num_hy = corridor_[i].size();
        for (size_t j = 0; j < num_hy; j++)
        {
            std::cout << i << "th corridor  \t" << j << "th polyhedron :" << std::endl;
            corridor_[i][j].show();
        }
    }
}

void auto_dock::solver_for_trajectory()
{
    flying_plan_[task_index_].get_path_constraint(corridor_[0], start_pt_[0], end_pt_[0]);
    std::cout << "Constraints  have been set for quadcopter." << std::endl;
    flying_plan_[task_index_].trajectory_solver_QP();
}

Eigen::VectorXd auto_dock::max_vector(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    Eigen::VectorXd out_v(v2.size());
    if (v1.size() == v2.size())
    {
        for (size_t i = 0; i < v1.size(); i++)
        {
            out_v[i] = max(v1[i], v2[i]);
        }
        return out_v;
    }
    else
    {
        ROS_ERROR("Function max_vector is wrong, scince the rows of the inputs are different!");
    } 
}

Eigen::VectorXd auto_dock::min_vector(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    Eigen::VectorXd out_v(v2.size());
    if (v1.size() == v2.size())
    {
        for (size_t i = 0; i < v1.size(); i++)
        {
            out_v[i] = min(v1[i], v2[i]);
        }
        return out_v;
    }
    else
    {
        ROS_ERROR("Function min_vector is wrong, scince the rows of the inputs are different!");
    } 
}

void auto_dock::get_path_cooridor(XmlRpc::XmlRpcValue &num_path,XmlRpc::XmlRpcValue &task_data)
{
    int n_path = num_path;
    start_pt_.clear();
    end_pt_.clear();
    corridor_.clear();
    end_effector_type_.clear();
    docking_pick_change_.clear();
    hold_time_manipulation_.clear();
    end_effector_switch_.clear();
    end_effector_switch_before_manip_.clear();
    end_effector_switch_after_manip_.clear();
    task_type_.clear();
    end_dock_offset_task_.clear();
    psi_angle_task_.clear();
    path_point  start_pt, end_pt;
    constraint temp_constr;
    vector<constraint> temp_corri;
    temp_corri.clear();

    for (size_t i = 0; i < n_path; i++)
    {
        start_pt.pos = Eigen::Vector3d(task_data[i] ["start_pt"]["pos"][0], task_data[i] ["start_pt"]["pos"][1], task_data[i] ["start_pt"]["pos"][2]);
        start_pt.vel = Eigen::Vector3d( task_data[i] ["start_pt"]["vel"][0], task_data[i] ["start_pt"]["vel"][1], task_data[i] ["start_pt"]["vel"][2]);
        start_pt.acc = Eigen::Vector3d( task_data[i] ["start_pt"]["acc"][0], task_data[i] ["start_pt"]["acc"][1], task_data[i] ["start_pt"]["acc"][2]);
        start_pt_.push_back(start_pt);
        end_pt.pos = Eigen::Vector3d( task_data[i] ["end_pt"]["pos"][0], task_data[i] ["end_pt"]["pos"][1], task_data[i] ["end_pt"]["pos"][2]);
        end_pt.vel = Eigen::Vector3d( task_data[i] ["end_pt"]["vel"][0], task_data[i] ["end_pt"]["vel"][1], task_data[i] ["end_pt"]["vel"][2]);
        end_pt.acc = Eigen::Vector3d( task_data[i] ["end_pt"]["acc"][0], task_data[i] ["end_pt"]["acc"][1], task_data[i] ["end_pt"]["acc"][2]);
        end_pt_.push_back(end_pt);
        temp_corri.clear();
        double hold_time = task_data[i]["hold_time"];
        hold_time_manipulation_.push_back(hold_time);
        int type_end = task_data[i]["end_eff_type"];
        end_effector_type_.push_back(type_end);
        int type_task = task_data[i]["task_type"];
        task_type_.push_back(type_task);
        bool flag_pick_change = task_data[i]["docking_pick_change"];
        docking_pick_change_.push_back(flag_pick_change);
        int end_effector_sw = task_data[i]["end_eff_switch"];
        end_effector_switch_.push_back(end_effector_sw);
        int end_effector_bf = task_data[i]["end_eff_switch_bef"];
        end_effector_switch_before_manip_.push_back(end_effector_bf);
        int end_effector_af = task_data[i]["end_eff_switch_aft"];
        end_effector_switch_after_manip_.push_back(end_effector_af);
        int num_poly = task_data[i]["num_polyhedron"];
        double end_offest_temp = task_data[i]["end_dock_offset"];
        end_dock_offset_task_.push_back(end_offest_temp);
        double psi_angle_temp = task_data[i]["yaw_angle"];
        psi_angle_task_.push_back(psi_angle_temp);
        
        for (size_t j = 0; j < num_poly; j++)
        {
            temp_constr.s_scale = task_data[i]["corridor"][j]["s_scale"];
            temp_constr.upper_vel = Eigen::Vector3d( task_data[i]["corridor"][j]["upper_vel"][0], task_data[i]["corridor"][j]["upper_vel"][1], task_data[i]["corridor"][j]["upper_vel"][2]);
            temp_constr.lower_vel = Eigen::Vector3d( task_data[i]["corridor"][j]["lower_vel"][0], task_data[i]["corridor"][j]["lower_vel"][1], task_data[i]["corridor"][j]["lower_vel"][2]);
            temp_constr.upper_acc = Eigen::Vector3d( task_data[i]["corridor"][j]["upper_acc"][0], task_data[i]["corridor"][j]["upper_acc"][1], task_data[i]["corridor"][j]["upper_acc"][2]);
            temp_constr.lower_acc = Eigen::Vector3d( task_data[i]["corridor"][j]["lower_acc"][0], task_data[i]["corridor"][j]["lower_acc"][1], task_data[i]["corridor"][j]["lower_acc"][2]);
            int row = task_data[i]["corridor"][j]["A_pos"]["row"];
            int col = task_data[i]["corridor"][j]["A_pos"]["col"];
            Eigen::MatrixXd A_temp(row,col);
            for (size_t k = 0; k < row; k++)
            {
                for (size_t u = 0; u < col; u++)
                {
                    A_temp(k,u) = task_data[i]["corridor"][j]["A_pos"]["data"][k*col+u];
                }
            }
            temp_constr.A_pos = A_temp;
            temp_constr.b_u_pos = Eigen::Vector3d( task_data[i]["corridor"][j]["b_u_pos"][0], task_data[i]["corridor"][j]["b_u_pos"][1], task_data[i]["corridor"][j]["b_u_pos"][2]);
            temp_constr.b_l_pos = Eigen::Vector3d( task_data[i]["corridor"][j]["b_l_pos"][0], task_data[i]["corridor"][j]["b_l_pos"][1], task_data[i]["corridor"][j]["b_l_pos"][2]);
            std::cout << j <<"th poly has been loaded." << std::endl;
            // temp.show();
            temp_corri.push_back(temp_constr);
        }
        corridor_.push_back(temp_corri);
    }
}

void auto_dock::task_manage( )
{
    if ((!task_flag_in_setp_) && (task_index_ < flying_plan_.size()))
    {
        flying_plan_[task_index_].get_path_constraint(corridor_[task_index_], start_pt_[task_index_], end_pt_[task_index_]);
        std::cout << "Constraints  have been set for quadcopter." << std::endl;
        flying_plan_[task_index_].trajectory_solver_QP();
        std::cout << "-------------------------" << std::endl;
        task_flag_in_setp_ = true;
    }
    
    if (task_flag_in_setp_ && flag_begin_dock_ )
    {
        switch (task_type_[task_index_])
        {
        case 0:
            task_index_ ++; 
            task_flag_in_setp_ = false;
            flag_begin_dock_ = false;
            flag_target_position_pos_ = false;
            break;

        case 1:
            if (docking_pick_change_[task_index_])// false --- pick, true --- change
            {
                if (!flag_docking_process_.flag_1)
                {
                    std_msgs::Int16 msg;
                    msg.data = last_tool_id_;
                    switch2docking.publish(msg);
                    std_msgs:: Float32MultiArray offset_temp;
                    offset_temp.data.push_back(last_offset_);
                    offset_temp.data.push_back(0.02f);
                    offset_end_pub.publish(offset_temp);
                    ROS_INFO("switch to docking (change). Tool is %d. offset is %f. rise up offset is %f", msg.data, offset_temp.data.at(0), offset_temp.data.at(1));
                    flag_docking_process_.flag_1 = true;
                }
                if (flag_target_position_pos_ && flag_docking_process_.flag_1 && (!flag_docking_process_.flag_2))
                {

                    std_msgs::String char2tool;
                    char temp_char[10];
                    sprintf(temp_char,"A%d%d%d",last_tool_id_, 0, 1);
                    char2tool.data = temp_char;
                    tool_chatter_pub.publish(char2tool);
                    std:: cout << "The electromagnet of tool No. "<< last_tool_id_<< " is closed! ----"<< temp_char <<std::endl;
                    if ((ros::Time::now() > time_hold_begin + ros::Duration(3.0))&& (ros::Time::now() < time_hold_begin + ros::Duration(3.5)))
                    {
                        std_msgs::Bool msg_tem;
                        msg_tem.data = true;
                        switch2flyingup.publish(msg_tem);
                        ROS_INFO("Switch to flying up.");
                    }
                    if (ros::Time::now() > time_hold_begin + ros::Duration(6.0))
                    {
                        flag_target_position_pos_ = false;
                        flag_docking_process_.flag_2 = true;
                        flag_docking_process_.time_2_position = ros::Time::now();
                    }
                }
                if (flag_docking_process_.flag_1 && flag_docking_process_.flag_2 && (!flag_docking_process_.flag_3))
                {
                    std_msgs::Int16 msg;
                    msg.data = end_effector_type_[task_index_];
                    last_tool_id_ = msg.data ;
                    switch2docking.publish(msg);
                    std_msgs:: Float32MultiArray offset_temp;
                    offset_temp.data.push_back(end_dock_offset_task_[task_index_ ]);
                    last_offset_ = end_dock_offset_task_[task_index_ ];
                    offset_temp.data.push_back(0.02f);
                    offset_end_pub.publish(offset_temp);
                    ROS_INFO("switch to docking (change). Tool is %d. offset is %f. rise up offset is %f", msg.data, offset_temp.data.at(0), offset_temp.data.at(1));
                    flag_docking_process_.flag_3 = true;
                }
                if (flag_target_position_pos_ &&flag_docking_process_.flag_1&& flag_docking_process_.flag_2 && flag_docking_process_.flag_3)
                {
                    std_msgs::String char2tool;
                    char temp_char[10];
                    sprintf(temp_char,"A%d%d%d",end_effector_type_[task_index_ ], 1, 1);
                    char2tool.data = temp_char;
                    tool_chatter_pub.publish(char2tool);
                    std:: cout << "The electromagnet of tool No. "<< end_effector_type_[task_index_ ]<< " is turned on! ----"<< temp_char <<std::endl;
                    if ((ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_])) && 
                    (ros::Time::now() < time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_] + 0.5)))
                    {
                        std_msgs::Bool msg_tem;
                        msg_tem.data = true;
                        switch2flyingup.publish(msg_tem);
                        ROS_INFO("Switch to flying up.");
                    }
                    if (ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_]+ 1.0))
                    {
                        task_index_ ++; 
                        task_flag_in_setp_ = false;
                        flag_begin_dock_ = false;
                        flag_target_position_pos_ = false;
                        flag_docking_process_.reset();
                        std_msgs::Bool msg_tem;
                        msg_tem.data = true;
                        switch2moving.publish(msg_tem);
                        ROS_INFO("Switch to moving.");
                    }
                }// docking 
            }
            else // false --- pick, true --- change
            {
                
                if ( !flag_docking_process_.flag_1)
                {
                    std_msgs::Int16 msg;
                    msg.data = end_effector_type_[task_index_];
                    last_tool_id_ = msg.data ;
                    switch2docking.publish(msg);
                    std_msgs::Float32MultiArray offset_temp;
                    offset_temp.data.push_back(end_dock_offset_task_[task_index_ ]);
                    last_offset_ = end_dock_offset_task_[task_index_ ];
                    offset_temp.data.push_back(0.02f);
                    offset_end_pub.publish(offset_temp);
                    ROS_INFO("switch to docking (install). Tool is %d. offset is %f. rise up offset is %f", msg.data, offset_temp.data.at(0), offset_temp.data.at(1));
                    flag_docking_process_.flag_1 = true;
                }
                if (flag_target_position_pos_ && (!flag_docking_process_.flag_2))
                {   
                    std_msgs::String char2tool;
                    char temp_char[10];
                    sprintf(temp_char,"A%d%d%d",end_effector_type_[task_index_ ], 1, 1);
                    char2tool.data = temp_char;
                    tool_chatter_pub.publish(char2tool);
                    std:: cout << "The electromagnet of tool No. "<< end_effector_type_[task_index_ ]<< " is turned on! ----"<< temp_char <<std::endl;
                    if ((ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_])) && 
                    (ros::Time::now() < time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_] + 0.5)))
                    {
                        std_msgs::Bool msg_tem;
                        msg_tem.data = true;
                        switch2flyingup.publish(msg_tem);
                        ROS_INFO("Switch to flying up.");
                    }
                    if (ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_]+ 1.0))
                    {
                        task_index_ ++; 
                        task_flag_in_setp_ = false;
                        flag_begin_dock_ = false;
                        flag_target_position_pos_ = false;
                        flag_docking_process_.reset();
                        std_msgs::Bool msg_tem;
                        msg_tem.data = true;
                        switch2moving.publish(msg_tem);
                        ROS_INFO("Switch to moving.");
                    }
                }
            }
            break;

        case 2:
            if (!flag_docking_process_.flag_1)
            {
                std_msgs::String char2tool;
                char temp_char[10];
                int a_switch = end_effector_switch_before_manip_[task_index_];
                sprintf(temp_char,"A%d%d%d",last_tool_id_, 1, a_switch);
                char2tool.data = temp_char;
                tool_chatter_pub.publish(char2tool);
                std:: cout << "No. "<< last_tool_id_ << " tool execution operation: ----"<< temp_char <<std::endl;

                std_msgs::Int16 msg;
                msg.data = end_effector_type_[task_index_];
                switch2docking.publish(msg);
                std_msgs:: Float32MultiArray offset_temp;
                offset_temp.data.push_back(end_dock_offset_task_[task_index_ ]);
                offset_temp.data.push_back(0.0f);
                offset_end_pub.publish(offset_temp);
                ROS_INFO("switch to manipulation. Target is %d. offset is %f", msg.data, offset_temp.data);
                flag_docking_process_.flag_1 = true;
            }
            if (flag_target_position_pos_)
            {
                // End-effector 
                if (ros::Time::now() < time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_]))
                {
                    std_msgs::String char2tool;
                    char temp_char[10];
                    int a_switch = end_effector_switch_[task_index_];
                    sprintf(temp_char,"A%d%d%d",last_tool_id_, 1, a_switch);
                    char2tool.data = temp_char;
                    tool_chatter_pub.publish(char2tool);
                    std:: cout << "No. "<< last_tool_id_ << " tool execution operation: ----"<< temp_char <<std::endl;
                }
                
                if ((ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_])) && 
                    (ros::Time::now() < time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_] + 0.5)))
                {
                    std_msgs::Bool msg_tem;
                    msg_tem.data = true;
                    switch2flyingup.publish(msg_tem);
                    ROS_INFO("Switch to flying up.");
                }

                if ((ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_] + 1.0)) && 
                    (ros::Time::now() < time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_] + 1.2)))
                {
                    std_msgs::String char2tool;
                    char temp_char[10];
                    int a_switch = end_effector_switch_after_manip_[task_index_];
                    sprintf(temp_char,"A%d%d%d",last_tool_id_, 1, a_switch);
                    char2tool.data = temp_char;
                    tool_chatter_pub.publish(char2tool);
                    std:: cout << "No. "<< last_tool_id_ << " tool execution operation: ----"<< temp_char <<std::endl;
                }

                if (ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_]+ 1.2))
                {

                    task_index_ ++;
                    task_flag_in_setp_ = false;
                    flag_begin_dock_ = false;
                    flag_target_position_pos_ = false;
                    flag_docking_process_.reset();
                    std_msgs::Bool msg_tem;
                    msg_tem.data = true;
                    switch2moving.publish(msg_tem);
                    ROS_INFO("Switch to moving.");
                }
            }
            break;
        case 3:
            if (!flag_docking_process_.flag_1)
            {
                flag_docking_process_.flag_1 = true;
                time_hold_begin = ros::Time::now();
            }
            if (flag_docking_process_.flag_1)
            {
                if ((ros::Time::now() -time_hold_begin).toSec() > hold_time_manipulation_[task_index_])
                {
                    task_index_ ++;
                    task_flag_in_setp_ = false;
                    flag_begin_dock_ = false;
                    flag_target_position_pos_ = false;
                    flag_docking_process_.flag_1 = false;
                    ROS_INFO("Holding is over.");
                }
            }
            break;

        case 4:// 
            if (!flag_docking_process_.flag_1)
            {
                std_msgs::Int16 msg;
                msg.data = end_effector_type_[task_index_];
                last_tool_id_ = msg.data ;
                switch2docking.publish(msg);
                std_msgs:: Float32MultiArray offset_temp;
                offset_temp.data.push_back(end_dock_offset_task_[task_index_ ]);
                offset_temp.data.push_back(0.02f);
                offset_end_pub.publish(offset_temp);
                ROS_INFO("switch to docking (unstall). Tool is %d. offset is %f. rise up offset is %f", msg.data, offset_temp.data.at(0), offset_temp.data.at(1));
                flag_docking_process_.flag_1 = true;
            }
            if (flag_target_position_pos_ && (!flag_docking_process_.flag_2))
            {   
                std_msgs::String char2tool;
                char temp_char[10];
                sprintf(temp_char,"A%d%d%d",end_effector_type_[task_index_ ], 0, 1);
                char2tool.data = temp_char;
                tool_chatter_pub.publish(char2tool);
                std:: cout << "The electromagnet of tool No. "<< end_effector_type_[task_index_ ]<< " is turned off! ----"<< temp_char <<std::endl;
                if (ros::Time::now() > time_hold_begin + ros::Duration(hold_time_manipulation_[task_index_]))
                {
                    task_index_ ++; 
                    task_flag_in_setp_ = false;
                    flag_begin_dock_ = false;
                    flag_target_position_pos_ = false;
                    flag_docking_process_.reset();
                    std_msgs::Bool msg_tem;
                    msg_tem.data = true;
                    switch2moving.publish(msg_tem);
                    ROS_INFO("Switch to moving.");
                }
            }
            break;
        default:
            break;
        }
    }
}

void auto_dock::test()
{
    ros::Time start, end;
    start = ros::Time::now();
    solver_for_trajectory();
    end = ros::Time::now();
    double cost_time = end.toSec() - start.toSec();
    std::cout << "Calculation time: "<< cost_time <<std::endl;
    std::cout << "Calculation has ended" <<std::endl;
    std_msgs::String char2tool;
    std::stringstream temp;
    temp <<  "A110";
    char2tool.data = temp.str();
    tool_chatter_pub.publish(char2tool);
    std:: cout << "Mechanical grab electromagnet is turned on!" <<std::endl;
}

void auto_dock::switch2moving_call(const std_msgs::Int16::ConstPtr &msg)
{
    if (msg->data)
    {
        task_index_ ++;
        flying_plan_[task_index_].get_path_constraint(corridor_[task_index_], start_pt_[task_index_], end_pt_[task_index_]);
        std::cout << "Constraints  have been set for quadcopter." << std::endl;
        flying_plan_[task_index_].trajectory_solver_QP();
        std::cout << "-------------------------" << std::endl;
        task_begin_flag_ = false;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "traj_solver");
    ros::NodeHandle auto_node;
    ROS_INFO("Read to solve trajectory!");
    auto_dock dock_object;
    ros::Rate rate(50.0);
    while(ros::ok())
    {
        dock_object.task_manage();
        ros::spinOnce();
        rate.sleep();
    }
}




