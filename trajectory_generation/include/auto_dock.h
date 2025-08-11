#ifndef AUTO_DOCK
#define AUTO_DOCK

#include "type_define.h"
#include <cstdio>
#include "end_effector_planning.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include"trajectory_solver.h"
#include "trajectory_generation/switch2moving_msg.h"

// behavior definitions
enum m_behavior {MOVE, DOCK_PICK,DOCK_CHANGE, MANIPULATION};

struct flag_docking_process
{
    bool flag_1 = false;
    bool flag_2 = false;
    bool flag_3 = false;
    bool flag_4 = false;
    bool flag_5 = false;
    ros::Time time_2_position;
    void reset()
    {
        flag_1 = false;
        flag_2 = false;
        flag_3 = false;
        flag_4 = false;
        flag_5 = false;
    }
};


class auto_dock
{
public:
    auto_dock(/* args */);
    void test();
    void task_manage( );
private:
    ros::NodeHandle nh_;
    ros::ServiceServer traj_result_server;
    ros::Publisher tool_chatter_pub;
    ros::Publisher switch2docking; 
    ros::Publisher switch2moving; 
    ros::Publisher switch2flyingup; 
    ros::Publisher offset_end_pub; 
    ros::Subscriber pos_arrive_sub;      


    void solver_for_trajectory();
    Eigen::VectorXd max_vector(Eigen::VectorXd v1, Eigen::VectorXd v2);
    Eigen::VectorXd min_vector(Eigen::VectorXd v1, Eigen::VectorXd v2);
    void time_align_for_end();
    void show_corridor();    

    bool traj_out_call(trajectory_generation::traj_out_msgRequest& request,trajectory_generation::traj_out_msgResponse& response);
    void pos_arrive_callback(const std_msgs::Bool::ConstPtr &msg );
    void switch2moving_call(const std_msgs::Int16::ConstPtr &msg);
    void get_path_cooridor(XmlRpc::XmlRpcValue &num_path,XmlRpc::XmlRpcValue &task_data);
    vector<trajectory_solver> flying_plan_;

    vector<vector<constraint>> corridor_;
    vector<vector<Eigen::Vector3d>> envir_;
    workspace workspace_;
    vector<path_point> start_pt_, end_pt_;

    bool task_begin_flag_ = false;
    int task_index_ = 0;
    vector <int> end_effector_type_;
    vector <bool> docking_pick_change_;
    vector <double> psi_angle_task_;
    vector <double> hold_time_manipulation_;
    vector <double> end_dock_offset_task_;
    vector <int> task_type_;
    vector <int> end_effector_switch_;
    vector <int> end_effector_switch_before_manip_;
    vector <int> end_effector_switch_after_manip_;
    m_behavior behavior_now_;
    int last_tool_id_;
    double last_offset_;

    double story_x_,story_y_,story_z_;

    bool task_flag_in_setp_ = false;
    bool flag_begin_dock_ = false;
    bool flag_target_position_pos_ = false;
    ros::Time time_hold_begin;
    double hold_time_docking_ = 5.0;

    flag_docking_process flag_docking_process_;
};




















#endif