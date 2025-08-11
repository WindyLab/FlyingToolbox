#ifndef _MY_LIB_H_
#define _MY_LIB_H_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include<vector>
#include "osqp/osqp.h"
#include <Eigen/Eigen>

using namespace std;
#define zero_ 1e-18

class VectorXd_array
{
private:
    c_float* temp;
public:
    VectorXd_array(Eigen::VectorXd in, c_float* out)
    {
        temp = new  c_float[in.size()];
        for (size_t i = 0; i < in.size(); i++)
        {
            temp[i] = in[i];
        }
        out = temp;
    }
};

 inline void matrix_upptriangular(Eigen::MatrixXd in, Eigen::MatrixXd& out)
 {
    out.resize(in.rows(), in.cols());
    out.setZero();
    for (size_t i = 0; i < in.rows(); i++)
    {
        for (size_t j = i; j < in.cols(); j++)
        {
            out(i,j) = in(i,j);
        }
    }
 }


inline void copy(Eigen::VectorXd in, c_float* out)
{
    for (size_t i = 0; i < in.size(); i++)
    {
        out[i] = in[i];
    }
}

inline void csc_combine(c_int A_nzmax, c_int A_n, c_int A_m, c_int* A_p, c_int* A_i, c_float* A_var,
    c_int B_nzmax, c_int B_n, c_int B_m, c_int* B_p, c_int* B_i, c_float* B_var,
     c_int& C_nzmax, c_int& C_n, c_int& C_m, c_int  p[], c_int ii[], c_float var[]){
    if (A_n != B_n) 
    {
        ROS_ERROR_STREAM("Conbine error!\n");
    }
    C_n = A_n;
    C_m = A_m + B_m;
    C_nzmax = A_nzmax + B_nzmax;
    var[25];
    p[9];
    ii[25];
    p[0] = 0;   
    
    for (size_t i = 0; i < A_n; i++)
    {
        
        p[i+1] = A_p[i+1] + B_p[i+1];
    }
    int index = 0;
    for (size_t i = 0; i < A_n; i++)
    {
        
        for (size_t j = 0; j < A_p[i+1]-A_p[i]; j++)
        {
            
            var[index]  = A_var[A_p[i]+j];
            ii[index]   = A_i[A_p[i]+j];
            index ++;
        }
        for (size_t j = 0; j < B_p[i+1]-B_p[i]; j++)
        {
           
            var[index]  = B_var[B_p[i]+j];
            ii[index]   = B_i[B_p[i]+j] + A_m;
            index ++;
        }
    }
}
inline bool csc_show(csc* C){
    c_float temp = 0;
    for (size_t i = 0; i < C->m; i++)
    {
        for (size_t j = 0; j < C->n; j++)
        {
            temp = 0;
            for (size_t t = 0; t < C->p[j+1]-C->p[j]; t++)
            {
                if (i==C->i[C->p[j]+t])
                {
                    temp = C->x[C->p[j]+t];
                } 
            }
            printf("%f\t",temp);
        }
        printf("\n");
    }
    return 1;
}

class matrix_to_csc
{
private:
    c_int count = 0;
    vector<double> var_vec;
    vector<c_int> raw_vec;
    vector<c_int> con_vec;
     void matrix2csc(Eigen::MatrixXd matrix )
     {
        con_vec.push_back(0);
        for (size_t i = 0; i < matrix.cols(); i++)
        {
            for (size_t j = 0; j < matrix.rows(); j++)
            {
                if (abs(matrix(j,i)) >zero_)
                {
                    count++;
                    var_vec.push_back(matrix(j,i));
                    raw_vec.push_back(j);
                }
            }
            con_vec.push_back(count);
        }
    }
public:
    matrix_to_csc(Eigen::MatrixXd matrix, csc& csc_mat){
        matrix2csc(matrix);
        csc_mat.nzmax = count;
        csc_mat.m = matrix.rows(); 
        csc_mat.n =  matrix.cols(); 
        csc_mat.nz = -1;
        csc_mat.p = &con_vec[0];
        csc_mat.i  = &raw_vec[0];
        csc_mat.x = &var_vec[0];
    } 
    ~matrix_to_csc(){};
};

inline void diagonal_expansion(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd& C)
{
    int m1 = A.rows(); 
    int m2 = B.rows(); 
    int n1 = A.cols(); 
    int n2 = B.cols();
    C.resize(m1+m2,n1+n2);
    C<<A, Eigen::MatrixXd::Zero(m1,n2),
            Eigen::MatrixXd::Zero(m2,n1),B;
}


inline void col_expansion(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd& C)
{
    int m1 = A.rows(); 
    int m2 = B.rows(); 
    int n1 = A.cols(); 
    int n2 = B.cols();
    if (m1==m2)
    {
        C.resize(m1,n1+n2);
        C<<A, B;
    }
    else
    {
        printf("Column_expansion is wrong! Rows of the two input matrice are different.\n");
    }
}

inline void row_expansion(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd& C)
{
    int m1 = A.rows(); 
    int m2 = B.rows(); 
    int n1 = A.cols(); 
    int n2 = B.cols();
    if (n1==n2)
    {
        C.resize(m1+m2,n1);
        C<<A, B;
    }
    else
    {
        printf("row_expansion is wrong!(%d, %d) Columns of the two input matrice are different.\n", n1,n2);
    }
    
}

inline void vector_expansion(Eigen::VectorXd in1, Eigen::VectorXd in2, Eigen::VectorXd& out)
{
    out.resize(in1.size()+in2.size());
    out<< in1,in2;
}

template<typename T>
inline bool vector_show(T vec[], int size){
    for (size_t i = 0; i < size; i++)
    {
        printf("%f\n",vec[i]);
    }
    return 1;
}
// factorial
inline int fac(int n)
{
	int f;
	if(n<0)
		printf("n<0,data error!");
	else if(n==0||n==1)
		f=1;
	else
		f=fac(n-1)*n;
	return(f);
}


inline int combinatorial(int n, int k) // for calculate n choose k combination problem
{
    return fac(n) / (fac(k) * fac(n - k));
};

inline int factorial(int n)    
{    
    if(n<0)    
        return(-1); /*Wrong value*/      
    if(n==0)    
        return(1);  /*Terminating condition*/    
    else    
    {    
        return(n*factorial(n-1));
    }    
}

template<typename _Matrix_Type_> 
inline _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = 
    std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
} 


inline void print_covex_hub(vector<Eigen::Vector3d> c_h)
{
    for (size_t i = 0; i < c_h.size(); i++)
    {
        std::cout << c_h[i][0] << "\t"<< c_h[i][1]<< "\t"<< c_h[i][2]<< std::endl;
    }
}




#endif
