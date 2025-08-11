#include"end_effector_planning.h"

end_effector_planning::end_effector_planning()
{
    settings  = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    data  = (OSQPData *)c_malloc(sizeof(OSQPData));
}

void end_effector_planning::get_objectives(pick_object obj,  double time_to_hold)
{
    obj_.clear();
    obj_.push_back(obj);
    end_pt_.pos = obj.pos;
    end_pt_.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
    end_pt_.acc = Eigen::Vector3d(0.0, 0.0, 0.0);
    end_pt_.show();
    time_to_hold_ = time_to_hold;
}

void end_effector_planning::get_initial_states(ini_end_effector ini_pt)
{
    ini_pt_.push_back(ini_pt);
    start_pt_ = ini_pt_[0].pt;
}

void  end_effector_planning::get_matrix_quadratic_form()
{
    int _poly_order_min = 8;
    int _poly_order_max = 12;
    if(bernstein_.setParam(_poly_order_min, _poly_order_max, opti_min_order_) == -1) 
    {
    ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
    }
    MatrixXd MQM = bernstein_.getMQM()[traj_order_];

    int n_segment = path_constraint_.size();
    std::cout << "n_segment = " << n_segment << endl;

    MatrixXd quad_m, quad_0;
    diagonal_expansion(MQM, MQM,quad_m);
    diagonal_expansion(quad_m, MQM,quad_m);

    for (size_t i = 0; i < n_segment; i++)
    {
        quad_0 = 1.0/(double) pow(path_constraint_[i].s_scale,  2*opti_min_order_ - 3) * quad_m;
        diagonal_expansion(quad_matrix_, quad_0, quad_matrix_);
    }
}

void  end_effector_planning::get_path_constraint( constraint path_constraint)
{
    path_constraint_.clear();
   path_constraint_.push_back(path_constraint);
}

void end_effector_planning::get_constraint_QP()
{
    int ctrl_num = traj_order_ + 1;
    int ctrl_3d_num = 3* ctrl_num;

    int n_segment = path_constraint_.size();
    A_matrix_.resize(0,0);
    low_vector_.resize(0);
    upp_vector_.resize(0);
    int row_cout = 0;

    Eigen::VectorXd vec_one= Eigen::VectorXd::Ones(ctrl_num);
    Eigen::MatrixXd matr_ctrl_num_ident = MatrixXd::Identity(ctrl_num,ctrl_num);

    for (size_t i = 0; i < n_segment; i++)
    {
        // for x,y,z
        int m1 = path_constraint_[i].A_pos.rows(); 
        int n1  = path_constraint_[i].A_pos.cols(); // n1=3, represents 3d trajectory
        if (n1!=3)
        {
            ROS_ERROR("Column of A is wrong!");
        }
        
        // Set b_l<=AX<=b_u for single cooridor
        Eigen::MatrixXd matr_temp(ctrl_num,ctrl_3d_num);//Note: 3 and N must be the same.
        
        Eigen::MatrixXd  A_old;
        Eigen::MatrixXd A_single(0,0);

       for (size_t j = 0; j < m1; j++)
       {
           matr_temp <<  path_constraint_[i].A_pos(j,0) * path_constraint_[i].s_scale * matr_ctrl_num_ident, 
                                           path_constraint_[i].A_pos(j,1)*path_constraint_[i].s_scale *matr_ctrl_num_ident, 
                                           path_constraint_[i].A_pos(j,2)*path_constraint_[i].s_scale *matr_ctrl_num_ident;
            A_old = A_single;
            A_single.resize((j+1)*ctrl_num,ctrl_3d_num);
            if (j == 0)
            {
                A_single = matr_temp;
            }
            else
            {
                A_single<<A_old,matr_temp;
            }
       }
       // add  b_l<=AX<=b_u into A_matrix_
       diagonal_expansion(A_matrix_, A_single, A_matrix_);

       row_cout = row_cout + m1*ctrl_num;
       
        for (size_t j = 0; j < m1; j++)
        {
            vector_expansion(low_vector_,  path_constraint_[i].b_l_pos[j]*vec_one,   low_vector_);
            vector_expansion(upp_vector_,  path_constraint_[i].b_u_pos[j]*vec_one, upp_vector_);
        }
    }

    Eigen::MatrixXd mat_tep;
    Eigen::MatrixXd mat_tep_1;
    Eigen::MatrixXd mat_tep_2(0,0);// 
    Eigen::MatrixXd Mat_v(ctrl_num - 1, ctrl_num);
    Eigen::MatrixXd Mat_a(ctrl_num - 2, ctrl_num);
    Mat_v.setZero();
    Mat_a.setZero();
    for (size_t i = 0; i < ctrl_num - 1; i++)
    { 
        Mat_v(i,i) = -1.0;
        Mat_v(i,i+1) =   1.0;
    }
    for (size_t i = 0; i < ctrl_num - 2; i++)
    { 
        Mat_a(i,i) = 1.0;
        Mat_a(i,i+1) =  -2.0;
        Mat_a(i,i+2) =  1.0;
    }
    for (size_t i = 0; i < n_segment; i++)
    {
        // vel
        diagonal_expansion(traj_order_*Mat_v, traj_order_*Mat_v, mat_tep);
        diagonal_expansion(mat_tep, traj_order_*Mat_v, mat_tep);
        mat_tep_1 = mat_tep;
        row_cout = row_cout + 3*(ctrl_num-1);
        for (size_t j = 0; j < 3; j++)
        {
            vector_expansion(low_vector_,  path_constraint_[i].lower_vel[j]*Eigen::VectorXd::Ones(ctrl_num-1),   low_vector_);
            vector_expansion(upp_vector_,  path_constraint_[i].upper_vel[j]*Eigen::VectorXd::Ones(ctrl_num-1), upp_vector_);
        }
        // acc
        double temp1 = (traj_order_*(traj_order_-1))/path_constraint_[i].s_scale;
        diagonal_expansion(temp1*Mat_a, temp1*Mat_a, mat_tep);
        diagonal_expansion(mat_tep, temp1*Mat_a, mat_tep);
        row_expansion(mat_tep_1, mat_tep, mat_tep_1);
        row_cout = row_cout + 3*(ctrl_num-2);
        for (size_t j = 0; j < 3; j++)
        {
            vector_expansion(low_vector_,  path_constraint_[i].lower_acc[j]*Eigen::VectorXd::Ones(ctrl_num-2),   low_vector_);
            vector_expansion(upp_vector_, path_constraint_[i].upper_acc[j]*Eigen::VectorXd::Ones(ctrl_num-2), upp_vector_);
        }
        diagonal_expansion(mat_tep_2, mat_tep_1, mat_tep_2);
    }
    row_expansion(A_matrix_, mat_tep_2, A_matrix_);

    Eigen::MatrixXd Mat_continue((n_segment-1)*3*3,n_segment*ctrl_3d_num);
    Eigen::VectorXd V_continue((n_segment-1)*3*3);
    V_continue.setZero();
    
    Mat_continue.setZero();
    int shit_index = 0;
    int num;
    for (size_t i = 0; i < n_segment - 1; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            num =  shit_index + (j+1)*ctrl_num - 1;
            Mat_continue(9*i+j, num) = 1.0 * path_constraint_[i].s_scale;
            num =  shit_index + ctrl_3d_num + j*ctrl_num;
            Mat_continue(9*i+j, num) = -1.0 * path_constraint_[i+1].s_scale;
        }
        for (size_t j = 0; j < 3; j++)
        {
            num =  shit_index + (j+1)*ctrl_num - 2;
            Mat_continue(9*i + 3+ j, num) = -1.0 ;
            num =  shit_index + (j+1)*ctrl_num - 1;
            Mat_continue(9*i + 3+ j, num) = 1.0 ;
            num =  shit_index + ctrl_3d_num + j*ctrl_num;
            Mat_continue(9*i + 3 + j, num) = 1.0 ;
            num =  shit_index + ctrl_3d_num + j*ctrl_num + 1;
            Mat_continue(9*i + 3 + j, num) = -1.0 ;
        }
        for (size_t j = 0; j < 3; j++)
        {
            num =  shit_index + (j+1)*ctrl_num - 3;
            Mat_continue(9*i + 6 + j, num) = 1.0 / path_constraint_[i].s_scale;
            num =  shit_index + (j+1)*ctrl_num - 2;
            Mat_continue(9*i + 6 + j, num) = -2.0 / path_constraint_[i].s_scale;
            num =  shit_index + (j+1)*ctrl_num - 1;
            Mat_continue(9*i + 6 + j, num) = 1.0 / path_constraint_[i].s_scale;
            num =  shit_index + ctrl_3d_num + j*ctrl_num;
            Mat_continue(9*i + 6 + j, num) = -1.0/ path_constraint_[i+1].s_scale;
            num =  shit_index + ctrl_3d_num + j*ctrl_num + 1;
            Mat_continue(9*i + 6 + j, num) = 2.0/ path_constraint_[i+1].s_scale;
            num =  shit_index + ctrl_3d_num + j*ctrl_num + 2;
            Mat_continue(9*i + 6 + j, num) = -1.0/ path_constraint_[i+1].s_scale;
        }
        shit_index += ctrl_3d_num;
        // std::cout<<"--------------Mat_continue---------------"<<std::endl;
        // std::cout<<Mat_continue<<std::endl;
    }
    
    row_expansion(A_matrix_, Mat_continue,  A_matrix_);
    vector_expansion(low_vector_, V_continue,   low_vector_);
    vector_expansion(upp_vector_, V_continue, upp_vector_);

    Eigen::MatrixXd Mat_path(18,n_segment*ctrl_3d_num);
    Mat_path.setZero();
    Eigen::VectorXd Vec_path(18);
    Vec_path.setZero();
    for (size_t i = 0; i < 3; i++)// 
    {
        num = i*ctrl_num;
        Mat_path(i, num) = 1.0 *  path_constraint_[0].s_scale;
    }
    for (size_t i = 0; i < 3; i++)// 
    {
        num = i*ctrl_num;
        Mat_path(i + 3, num) = - 1.0 *  traj_order_;
        num = i*ctrl_num + 1;
        Mat_path(i + 3, num) =   1.0 *  traj_order_;
    }
    for (size_t i = 0; i < 3; i++)// 
    {
        num = i*ctrl_num;
        Mat_path(i + 6, num) =   1.0 *  traj_order_ * (traj_order_ - 1) / path_constraint_[0].s_scale;
        num = i*ctrl_num + 1;
        Mat_path(i + 6, num) =   -2.0 *  traj_order_ * (traj_order_ - 1) / path_constraint_[0].s_scale;
        num = i*ctrl_num + 2;
        Mat_path(i + 6, num) =   1.0 *  traj_order_ * (traj_order_ - 1) / path_constraint_[0].s_scale;
    }
    int end_element = path_constraint_.size() - 1;
    for (size_t i = 0; i < 3; i++)// 
    {
        num = n_segment*ctrl_3d_num - 1 - (2 -i) * ctrl_num;
        Mat_path(i + 9, num) = 1.0 *  path_constraint_[end_element].s_scale;
    }
    for (size_t i = 0; i < 3; i++)// 
    {
        num = n_segment*ctrl_3d_num - 1 - (2-i) * ctrl_num -1;
        Mat_path(i + 12, num) = - 1.0*traj_order_ ;
        num = n_segment*ctrl_3d_num - 1 - (2-i) * ctrl_num;
        Mat_path(i + 12, num) =   1.0*traj_order_;
    }
    for (size_t i = 0; i < 3; i++)// 
    {
        num = n_segment*ctrl_3d_num - 1 - (2-i) * ctrl_num -2;
        Mat_path(i + 15, num) =   1.0 *  traj_order_ * (traj_order_ - 1)/path_constraint_[end_element].s_scale;
        num = n_segment*ctrl_3d_num - 1 - (2-i) * ctrl_num -1;
        Mat_path(i + 15, num) =  -2.0 *  traj_order_ * (traj_order_ - 1)/path_constraint_[end_element].s_scale;
        num = n_segment*ctrl_3d_num - 1 - (2-i) * ctrl_num;
        Mat_path(i + 15, num) =   1.0 *  traj_order_ * (traj_order_ - 1)/path_constraint_[end_element].s_scale;
    }
    Vec_path << start_pt_.pos, start_pt_.vel, start_pt_.acc, end_pt_.pos, end_pt_.vel, end_pt_.acc;
    row_expansion(A_matrix_, Mat_path,  A_matrix_);
    vector_expansion(low_vector_, Vec_path,   low_vector_); 
    vector_expansion(upp_vector_, Vec_path, upp_vector_);
    
    // 
    Eigen::Vector3d w_min, w_max;
    w_min = obj_[0].rotation *(  workspace_[0].b_l_w + delta_offset_);
    w_max = obj_[0].rotation *(  workspace_[0].b_u_w + delta_offset_);
    
    Eigen::MatrixXd mat_geo;
    mat_geo.setIdentity(3*(traj_order_+1),3*(traj_order_+1));
    Eigen::VectorXd vec_geo_l(3*(traj_order_+1)), vec_geo_u(3*(traj_order_+1));
    for (size_t j = 0; j < traj_order_+1; j++)
    {
        vec_geo_l[  j + 0* (traj_order_+1)] = w_min[0] + bezier_flying_base_.coef_x[j];
        vec_geo_u[j + 0* (traj_order_+1)] = w_max[0] + bezier_flying_base_.coef_x[j];
    }
    for (size_t j = 0; j < traj_order_+1; j++)
    {
        vec_geo_l[  j + 1* (traj_order_+1)] = w_min[1] + bezier_flying_base_.coef_y[j];
        vec_geo_u[j + 1* (traj_order_+1)] = w_max[1] + bezier_flying_base_.coef_y[j];
    }
    for (size_t j = 0; j < traj_order_+1; j++)
    {
        vec_geo_l[  j + 2* (traj_order_+1)] = w_min[2] + bezier_flying_base_.coef_z[j];
        vec_geo_u[j + 2* (traj_order_+1)] = w_max[2] + bezier_flying_base_.coef_z[j];
    }
    
    // row_expansion(A_matrix_, mat_geo,  A_matrix_);
    // vector_expansion(low_vector_, vec_geo_l,   low_vector_);
    // vector_expansion(upp_vector_, vec_geo_u, upp_vector_);

    
    // 
    int col_temp = A_matrix_.cols();
    Eigen::MatrixXd Mat_grip(5,col_temp);
    Eigen::VectorXd Vec_grip_u(5);
    Eigen::VectorXd Vec_grip_l(5);
    Mat_grip.setZero();
    Vec_grip_u.setZero();
    Vec_grip_l.setZero();

    VectorXd CList   = bernstein_.getC()[traj_order_];
    double t = (path_constraint_[end_element].s_scale  - dt_last_step)/path_constraint_[end_element].s_scale ;
    
    for (size_t i = 0; i < (traj_order_ +1); i++)
    {
        Mat_grip(0, col_temp - 3*(traj_order_ + 1) + i) =  path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
        Mat_grip(0, col_temp - 1*(traj_order_ + 1) + i) =  tan_gamma * path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
    }
    Vec_grip_u[0]  = end_pt_.pos[0] + tan_gamma * end_pt_.pos[2];
    Vec_grip_l[0]  = -OSQP_INFTY;
    // std:: cout << "---------------------------" << std::endl;
    // std:: cout << Vec_grip_u << std::endl;
    // std:: cout << Vec_grip_l << std::endl;

    for (size_t i = 0; i < (traj_order_ +1); i++)
    {
        Mat_grip(1, col_temp - 3*(traj_order_ + 1) + i) =  path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
        Mat_grip(1, col_temp - 1*(traj_order_ + 1) + i) =  -tan_gamma * path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
    }
    Vec_grip_u[1]  = OSQP_INFTY;
    Vec_grip_l[1]  = end_pt_.pos[0] - tan_gamma * end_pt_.pos[2];

    for (size_t i = 0; i < (traj_order_ +1); i++)
    {
        Mat_grip(2, col_temp - 2*(traj_order_ + 1) + i) =  path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
        Mat_grip(2, col_temp - 1*(traj_order_ + 1) + i) =  tan_gamma * path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
    }
    Vec_grip_u[2]  = end_pt_.pos[1] + tan_gamma * end_pt_.pos[2];
    Vec_grip_l[2]  = -OSQP_INFTY;

    for (size_t i = 0; i < (traj_order_ +1); i++)
    {
        Mat_grip(3, col_temp - 2*(traj_order_ + 1) + i) =  path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
        Mat_grip(3, col_temp - 1*(traj_order_ + 1) + i) =  -tan_gamma * path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
    }
    Vec_grip_u[3]  = OSQP_INFTY;
    Vec_grip_l[3]  =  end_pt_.pos[1] - tan_gamma * end_pt_.pos[2];

    // for (size_t i = 0; i < (traj_order_ +1); i++)
    // {
    //     Mat_grip(4, col_temp - 1*(traj_order_ + 1) + i) =  path_constraint_[end_element].s_scale * CList(i) * pow(t, i) * pow((1 - t), (traj_order_ - i) );
    // }
    // Vec_grip_u[4]  = end_pt_.pos[2] - dl_grap;
    // Vec_grip_l[4]  = -OSQP_INFTY;

    // std:: cout << Mat_grip << std::endl;

    row_expansion(A_matrix_, Mat_grip,  A_matrix_);
    vector_expansion(low_vector_, Vec_grip_l,   low_vector_);
    vector_expansion(upp_vector_, Vec_grip_u, upp_vector_);
}

void end_effector_planning::trajectory_solver_QP()
{
    get_matrix_quadratic_form();
    get_constraint_QP();
    Eigen::VectorXd temp_q(A_matrix_.cols());
    temp_q.setZero();
    QP_solver(quad_matrix_, temp_q);
    std::cout << "------------0th iteration-----------" << std::endl;
    int count_max = 0;
    while (collision_check())
    {
        Eigen::MatrixXd Q_temp = quad_matrix_ + Q_col_;
        temp_q = temp_q + q_col_;
        QP_solver(Q_temp, temp_q);
        count_max++;
        std::cout << "------------th" << count_max << "iteration-----------" << std::endl;
        if (count_max == 150)
        {
            std::cout << "------------Unable to avoid obstacles!-----------" << std::endl;
            break;
        }
    }
    bezier_3d_[0].show();
}

void end_effector_planning::QP_solver(Eigen::MatrixXd quad_matrix, Eigen::VectorXd temp_q)
{
    csc temp_Q;
    Eigen::MatrixXd quad_matrix_temp;
    matrix_upptriangular(quad_matrix, quad_matrix_temp);
    matrix_to_csc trans_mat_csc(quad_matrix_temp,temp_Q);
    csc temp_A;
    matrix_to_csc trans_mat_csc_2(A_matrix_, temp_A);
    

    // Exitflag
    c_int exitflag = 0;

    // Populate data
    if (data) {
        data->n = A_matrix_.cols(); // 
        data->m = A_matrix_.rows();  
        data->P = &temp_Q; 
        data->q = new c_float[temp_q.size()];
        for (size_t i = 0; i < temp_q.size(); i++)
        {
            data->q[i] = temp_q[i];
        }
        data->A = &temp_A; 
        data->l = new c_float[low_vector_.size()];
        for (size_t i = 0; i < low_vector_.size(); i++)
        {
            data->l[i] = low_vector_[i];
        }
        data->u = new c_float[upp_vector_.size()];
        for (size_t i = 0; i < upp_vector_.size(); i++)
        {
            data->u[i] = upp_vector_[i];
        }
    }

    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);
    //   path_constraint_.size();
    bezier_3d_.resize(path_constraint_.size());
    ros::Time Now_time = ros::Time::now();

    for (size_t i = 0; i < path_constraint_.size(); i++)
    {
        bezier_3d_[i].scale  = path_constraint_[i].s_scale;
        bezier_3d_[i].order = traj_order_;
        bezier_3d_[i].times = ini_pt_[i].time_begin;
        Now_time = Now_time +   ros::Duration( bezier_3d_[i].scale);
        bezier_3d_[i].coef_x = new double[traj_order_+1];
        for (size_t j = 0; j < traj_order_+1; j++)
        {
            bezier_3d_[i].coef_x[j] = work->solution->x[i*3*(traj_order_+1) + j];
        }
        bezier_3d_[i].coef_y = new double[traj_order_+1];
        for (size_t j = 0; j < traj_order_+1; j++)
        {
            bezier_3d_[i].coef_y[j] = work->solution->x[i*3*(traj_order_+1) + (traj_order_+1) + j];
        }
        bezier_3d_[i].coef_z = new double[traj_order_+1];
        for (size_t j = 0; j < traj_order_+1; j++)
        {
            bezier_3d_[i].coef_z[j] = work->solution->x[i*3*(traj_order_+1) + 2*(traj_order_+1) + j];
        }
    }
}

void end_effector_planning::trajectory_out(ros::Time timeNow, path_point& traj_sp)
{
    if (timeNow.toSec() >= bezier_3d_[0].times.toSec()  )
    {
        bezier_out(timeNow, bezier_3d_[0], traj_sp);
    }
}

void end_effector_planning::time_align(ros::Time time_in)
{
    if (bezier_3d_.size() == 1)
    {
        bezier_3d_[0].times = time_in;
    }
    else
    {
        ROS_ERROR("End-effector planning is wrong at first step!"); // set the task time based on beginning time.
    }
}

bool end_effector_planning::bezier_out(ros::Time timeNow, bezier bezier_in, path_point& traj_sp)
{
    VectorXd CList   = bernstein_.getC()[bezier_in.order];
    VectorXd CvList = bernstein_.getC_v()[bezier_in.order];
    VectorXd CaList = bernstein_.getC_a()[bezier_in.order];

    traj_sp.pos.setZero();
    traj_sp.vel.setZero();
    traj_sp.acc.setZero();

    double t =  (timeNow.toSec() - bezier_in.times.toSec())/bezier_in.scale;
    if (t >1)
    {
        traj_sp.pos[0] =  bezier_in.scale *bezier_in.coef_x[bezier_in.order] ;
        traj_sp.pos[1] =  bezier_in.scale *bezier_in.coef_y[bezier_in.order] ;
        traj_sp.pos[2] =  bezier_in.scale *bezier_in.coef_z[bezier_in.order] ;
        traj_sp.vel[0] =  bezier_in.order * (bezier_in.coef_x[bezier_in.order] - bezier_in.coef_x[bezier_in.order-1]) ;
        traj_sp.vel[1] =  bezier_in.order * (bezier_in.coef_y[bezier_in.order] - bezier_in.coef_y[bezier_in.order-1]) ;
        traj_sp.vel[2] =  bezier_in.order * (bezier_in.coef_z[bezier_in.order] - bezier_in.coef_z[bezier_in.order-1]) ;
        traj_sp.acc[0]=  1.0 / bezier_in.scale * bezier_in.order * (bezier_in.order - 1) 
                                       * (bezier_in.coef_x[bezier_in.order]  - 2*bezier_in.coef_x[bezier_in.order-1]  + bezier_in.coef_x[bezier_in.order-2] );
        traj_sp.acc[1]=  1.0 / bezier_in.scale * bezier_in.order * (bezier_in.order - 1) 
                                       * (bezier_in.coef_y[bezier_in.order]  - 2*bezier_in.coef_y[bezier_in.order-1]  + bezier_in.coef_y[bezier_in.order-2] );
        traj_sp.acc[2]=  1.0 / bezier_in.scale * bezier_in.order * (bezier_in.order - 1) 
                                       * (bezier_in.coef_z[bezier_in.order]  - 2*bezier_in.coef_z[bezier_in.order-1]  + bezier_in.coef_z[bezier_in.order-2] );
        return 0;
    }
    else
    {
        for (size_t i = 0; i < (bezier_in.order+1); i++)
        {
            traj_sp.pos[0]  += bezier_in.scale *bezier_in.coef_x[i] * CList(i) * pow(t, i) * pow((1 - t), (bezier_in.order - i) );
            traj_sp.pos[1]  += bezier_in.scale *bezier_in.coef_y[i] * CList(i) * pow(t, i) * pow((1 - t), (bezier_in.order - i) );
            traj_sp.pos[2]  += bezier_in.scale *bezier_in.coef_z[i] * CList(i) * pow(t, i) * pow((1 - t), (bezier_in.order - i) );
            if (i<bezier_in.order)
            {
                traj_sp.vel[0] += CvList(i) * bezier_in.order * (bezier_in.coef_x[i+1] - bezier_in.coef_x[i]) * pow(t, i) * pow((1 - t), (bezier_in.order - 1 - i) );
                traj_sp.vel[1] += CvList(i) * bezier_in.order * (bezier_in.coef_y[i+1] - bezier_in.coef_y[i]) * pow(t, i) * pow((1 - t), (bezier_in.order - 1 - i) );
                traj_sp.vel[2] += CvList(i) * bezier_in.order * (bezier_in.coef_z[i+1] - bezier_in.coef_z[i]) * pow(t, i) * pow((1 - t), (bezier_in.order - 1 - i) );
            }
            if (i<bezier_in.order-1)
            {
                traj_sp.acc[0] += 1.0 / bezier_in.scale * CaList(i) * bezier_in.order * (bezier_in.order - 1) 
                                                * (bezier_in.coef_x[i+2]  - 2*bezier_in.coef_x[i+1]  + bezier_in.coef_x[i] ) * pow(t, i) * pow((1 - t), (bezier_in.order - 2 - i) );
                traj_sp.acc[1] += 1.0 / bezier_in.scale * CaList(i) * bezier_in.order * (bezier_in.order - 1) 
                                                * (bezier_in.coef_y[i+2]  - 2*bezier_in.coef_y[i+1]  + bezier_in.coef_y[i] ) * pow(t, i) * pow((1 - t), (bezier_in.order - 2 - i) );
                traj_sp.acc[2] += 1.0 / bezier_in.scale * CaList(i) * bezier_in.order * (bezier_in.order - 1) 
                                                * (bezier_in.coef_z[i+2]  - 2*bezier_in.coef_z[i+1]  + bezier_in.coef_z[i] ) * pow(t, i) * pow((1 - t), (bezier_in.order - 2 - i) );
            }
        }
        return 1;
    }
}

bool end_effector_planning::collision_check()
{
    // 
    for (size_t i = 0; i < obs_set_.size(); i++)
    {
        obs_set_[i].term_left = Eigen::Vector3d(0.0, 0.0, 0.0);
        obs_set_[i].term_right = Eigen::Vector3d(0.0, 0.0, 0.0);
        obs_set_[i].is_collision = false;
    }
    int max_check_step = 200;
    double dtime_step =  path_constraint_[0].s_scale / double(max_check_step);
    for (size_t i = 0; i < max_check_step; i++)
    {
        ros::Time time_col = bezier_3d_[0].times + ros::Duration(i*dtime_step);
        gjk_for_traj(time_col);
    }

    for (size_t i = 0; i < obs_set_.size(); i++)
    {
        if (obs_set_[i].was_collision)
        {
            if (obs_set_[i].is_collision)
            {
                double lammda_scale = 6.0;
                obs_set_[i].mirr_pos = (1+lammda_scale)*(obs_set_[i].term_left + obs_set_[i].term_right) - lammda_scale*obs_set_[i].center;
                Eigen::Vector3d temp_term = obs_set_[i].term_left - obs_set_[i].term_right;
                double tem_m = (temp_term.norm() < 0.01f)? 0.01f:temp_term.norm() ;
                obs_set_[i].lammda = obs_set_[i].lammda + 200.2* tem_m;
                Eigen::Vector3d length_collsition = obs_set_[i].term_left - obs_set_[i].term_right;
                std::cout << "length collision :" << std::endl;
                std::cout << length_collsition.norm() << std::endl;
            }
        }
    }
    
    Q_col_.resize(quad_matrix_.rows(),quad_matrix_.cols());
    Q_col_.setZero();
    q_col_.resize(quad_matrix_.rows());
    q_col_.setZero();

    for (size_t i = 0; i < obs_set_.size(); i++)
    {
        if (obs_set_[i].was_collision)
        {
            Q_col_ = Q_col_ + obs_set_[i].lammda * MatrixXd::Identity(quad_matrix_.rows(),quad_matrix_.cols());
            Eigen::VectorXd q_temp(quad_matrix_.rows());
            for (size_t j = 0; j < traj_order_ +1; j++)
            {
                q_temp[j] = -obs_set_[i].lammda *2*obs_set_[i].mirr_pos[0];
            }
            for (size_t j = 0; j < traj_order_ +1; j++)
            {
                q_temp[j + traj_order_ +1 ] =  -obs_set_[i].lammda *2*obs_set_[i].mirr_pos[1];
            }
            for (size_t j = 0; j < traj_order_ +1; j++)
            {
                q_temp[j + 2*traj_order_ +2 ] =  -obs_set_[i].lammda *2*obs_set_[i].mirr_pos[2];
            }
            q_col_ = q_col_ + q_temp;
        }
    } 
    
    for (size_t i = 0; i < obs_set_.size(); i++)
    {
        if (obs_set_[i].is_collision)
        {
            return true;
        }
    }
    return false;
}

void end_effector_planning::gjk_for_traj(ros::Time time_in)
{
    path_point p_body,p_end;
    bezier_out(time_in, bezier_flying_base_, p_body);
    // p_body.show();
    bezier_out(time_in, bezier_3d_[0], p_end);
    // p_end.show();
    vector<Eigen::Vector3d> vertices;
    vertices.clear();
    double r_S = 0.65, r_C = 0.06, l_C = 0.06;
    Eigen::Vector3d temp;
    // point 1
    temp = p_body.pos + obj_[0].rotation*rotation_delta_b_*Eigen::Vector3d(r_S*cos((1+2)/3*pi),r_S*sin((1+2)/3*pi), 0);
    vertices.push_back(temp);
    // point 2
    temp = p_body.pos + obj_[0].rotation*rotation_delta_b_*Eigen::Vector3d(r_S*cos((1+2*2)/3*pi),r_S*sin((1+2*2)/3*pi), 0);
    vertices.push_back(temp);
    // point 3
    temp = p_body.pos +  obj_[0].rotation*rotation_delta_b_*Eigen::Vector3d(r_S*cos((1+2*3)/3*pi),r_S*sin((1+2*3)/3*pi), 0);
    vertices.push_back(temp);
    // point 4
    temp = p_end.pos +  obj_[0].rotation*rotation_delta_b_*Eigen::Vector3d(r_C*cos((1+2*1)/3*pi),r_C*sin((1+2*1)/3*pi), l_C);
    vertices.push_back(temp);
    // point 5
    temp = p_end.pos +  obj_[0].rotation*rotation_delta_b_*Eigen::Vector3d(r_C*cos((1+2*2)/3*pi),r_C*sin((1+2*2)/3*pi), l_C);
    vertices.push_back(temp);
    // point 6
    temp = p_end.pos +  obj_[0].rotation*rotation_delta_b_*Eigen::Vector3d(r_C*cos((1+2*3)/3*pi),r_C*sin((1+2*3)/3*pi), l_C);
    vertices.push_back(temp);

    for (size_t i = 0; i < envir_.size(); i++)
    {
        vector<shape> ts1;
        ts1.resize(envir_[i].size());
        for (size_t j = 0; j < ts1.size(); j++)
        {
            ts1[j].x = envir_[i][j][0];
            ts1[j].y = envir_[i][j][1];
            ts1[j].z = envir_[i][j][2];
        }
        vector<shape> cs1_1;
        cs1_1.resize(vertices.size());
        for (size_t j = 0; j < cs1_1.size(); j++)
        {
            cs1_1[j].x = vertices[j][0];
            cs1_1[j].y = vertices[j][1];
            cs1_1[j].z = vertices[j][2];
        }
        int dim_ts = envir_[i].size();
        int dim_cs1_1 = vertices.size();
        if (gjk(&ts1[0], &cs1_1[0], dim_ts, dim_cs1_1))
        {
           if (!(obs_set_[i].term_left[0] == 0.0 && obs_set_[i].term_left[1] == 0.0 && obs_set_[i].term_left[2] == 0.0 ) )
           {
                obs_set_[i].term_right = p_end.pos;
           }
           else
           {
                obs_set_[i].is_collision = true;
                obs_set_[i].was_collision = true;
                obs_set_[i].term_left = p_end.pos;
                obs_set_[i].term_right = p_end.pos;
                obs_set_[i].center = Eigen::Vector3d(0.0,0.0,0.0);
                for (int j = 0;  j< envir_[i].size(); j++)
                {
                    obs_set_[i].center = obs_set_[i].center  + envir_[i][j];
                }
                 obs_set_[i].center = (1/envir_[i].size()) * obs_set_[i].center;
           }
        } 
    }
}

