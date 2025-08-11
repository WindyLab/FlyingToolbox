#include <ros/ros.h>
#include "bezier_base.h"
#include "mylib.h"
#include "osqp/osqp.h"
#include "trajectory_generation/traj_solver_msg.h"
#include "trajectory_generation/traj_out_msg.h"





// bezier result
struct bezier{
  int  order;         // order
  double scale;       // scarl
  double*   coef;     // coefficient
  ros::Time times;    // begin time 
} ;


class trajectory_gen
{
public:
    
    trajectory_gen();


    ros::NodeHandle nh;
    ros::ServiceServer traj_solver_server;
    ros::ServiceServer traj_result_server;

    double* beziersolver10(c_float s_scale, c_float upperPosition,c_float lowerPosition,c_float upperVelocity,
                            c_float lowerVelocity,c_float upperAccelera,c_float lowerAccelera,c_float x_0,c_float x_n,c_float v_0,c_float v_n);
    double* beziersolver8(c_float s_scale, c_float upperPosition,c_float lowerPosition,c_float upperVelocity,
                            c_float lowerVelocity,c_float upperAccelera,c_float lowerAccelera,c_float x_0,c_float x_n,c_float v_0,c_float v_n);
    float* bezierout(bezier _bezier, ros::Time timeNow);
    bool traj_solver_call(trajectory_generation::traj_solver_msgRequest& request,trajectory_generation::traj_solver_msgResponse& response);
    bool traj_out_call(trajectory_generation::traj_out_msgRequest& request,trajectory_generation::traj_out_msgResponse& response);
    bezier _bezier_x,_bezier_y,_bezier_z;
                          

private:
    Bernstein _bernstein;
};
