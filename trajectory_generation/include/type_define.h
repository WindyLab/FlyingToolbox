#ifndef TYPE_DEFINE
#define TYPE_DEFINE
#include <ros/ros.h>
#include<vector>
#include<iostream>
#include<cstring>
#include<stddef.h>
#include <Eigen/Eigen>
#include "bezier_base.h"
#include "mylib.h"
#include <stdio.h>
#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "osqp/osqp.h"
/*! \brief Rename the float type used in lib

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;
///Pre-allocated std::vector for Eigen using vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;

// bezier result
struct bezier{
  int  order;         // order
  double scale;       // scarl
  double*   coef_x;     // coefficient
  double*   coef_y;     // coefficient
  double*   coef_z;     // coefficient
  ros::Time times;    // begin time 
  bezier(){};
  bezier(int order_in, double scale_in, ros::Time times_in)
  {
    order = order_in;
    scale = scale_in;
    times = times_in;
    coef_x = new double[order+1];
    coef_y = new double[order+1];
    coef_z = new double[order+1];
  }
  void show()
    {
      std::cout << "---bezier:---"  <<std::endl;
      std::cout << "order:"  << order <<std::endl;
      std::cout << "scale:"  << scale <<std::endl;
      std::cout << "coef_x:"  <<std::endl;
      for (size_t i = 0; i <= order; i++)
      {
        printf("%f\t",coef_x[i]);
      }
      printf("\n");
      std::cout << "coef_y:"  <<std::endl;
      for (size_t i = 0; i <= order; i++)
      {
        printf("%f\t",coef_y[i]);
      }
      printf("\n");
       std::cout << "coef_z:"  <<std::endl;
      for (size_t i = 0; i <= order; i++)
      {
        printf("%f\t",coef_z[i]);
      }
      printf("\n");
    }  
} ;

struct constraint
{
    double s_scale;
    Eigen::Vector3d upper_vel;
    Eigen::Vector3d lower_vel;
    Eigen::Vector3d upper_acc;
    Eigen::Vector3d lower_acc;
    Eigen::MatrixXd A_pos;
    Eigen::VectorXd b_u_pos;
    Eigen::VectorXd b_l_pos;
    void show()
    {
        std::cout << "s_scale:" <<std::endl;
        std::cout << s_scale <<std::endl;
        std::cout << "upper_vel:" <<std::endl;
        std::cout << upper_vel <<std::endl;
        std::cout << "lower_vel:" <<std::endl;
        std::cout << lower_vel <<std::endl;
        std::cout << "upper_acc:" <<std::endl;
        std::cout << upper_acc <<std::endl;
        std::cout << "lower_acc:" <<std::endl;
        std::cout << lower_acc <<std::endl;
        std::cout << "A_pos:" <<std::endl;
        std::cout << A_pos <<std::endl;
        std::cout << "b_u_pos:" <<std::endl;
        std::cout << b_u_pos <<std::endl;
        std::cout << "b_l_pos:" <<std::endl;
        std::cout << b_l_pos <<std::endl;
    }
    bool is_in(Eigen::Vector3d pt)
    {
      Eigen::VectorXd b = A_pos * pt;
      bool flag_in = true;
      for (size_t i = 0; i < b.size(); i++)
      {
        flag_in = flag_in && (b[i] <= b_u_pos[i] ) && (b[i] >= b_l_pos[i] ) ;
      }
      return flag_in;
    }
};

struct path_point
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    void show()
    {
        std::cout << "pos:" <<std::endl;
        std::cout << pos <<std::endl;
        std::cout << "vel:" <<std::endl;
        std::cout << vel <<std::endl;
        std::cout << "acc:" <<std::endl;
        std::cout << acc <<std::endl;
    }
    path_point& operator=(const path_point& cpy)
    {
        pos = cpy.pos;
        vel = cpy.vel;
        acc = cpy.acc;
        return *this;
    }
};

struct pick_object
{
  Eigen::Vector3d pos;
  double constrant_angle;
  Eigen::Matrix3d rotation;
  void get_euler(Eigen::Vector3d euler)
  {
    rotation(0,0) = cos(euler(1))*cos(euler(2));
    rotation(0,1) = sin(euler(1))*sin(euler(0))*cos(euler(2))-cos(euler(0))*sin(euler(2));
    rotation(0,2) = sin(euler(1))*cos(euler(0))*cos(euler(2))+sin(euler(0))*sin(euler(2));
    rotation(1,0) = cos(euler(1))*sin(euler(2));
    rotation(1,1) = sin(euler(1))*sin(euler(0))*sin(euler(2))+cos(euler(0))*cos(euler(2));
    rotation(1,2) = -sin(euler(0))*cos(euler(2))+cos(euler(0))*sin(euler(1))*sin(euler(2));
    rotation(2,0) = -sin(euler(1));
    rotation(2,1) = -sin(euler(0))*cos(euler(1));
    rotation(2,2) = cos(euler(0))*cos(euler(1));
  }
};

struct workspace
{
  Eigen::MatrixXd A_w;
  Eigen::VectorXd b_u_w;
  Eigen::VectorXd b_l_w;
  Eigen::Vector3d center;
};

struct ini_end_effector
{
    ros::Time time_begin;
    path_point pt;
};

#endif