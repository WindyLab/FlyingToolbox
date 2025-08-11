#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>

// Motion model: state prediction
void motion_model(const Eigen::MatrixXd& s_est, double dt, Eigen::MatrixXd& s_pre, Eigen::MatrixXd& F) {
    F = Eigen::MatrixXd::Identity(6,6);
    F(0,3) = dt;
    F(1,4) = dt;
    F(2,5) = dt;
    s_pre = F * s_est;
    // You can insert more physical models here
}

// Observation model: from state to measurement
void observation_model(const Eigen::MatrixXd& s_pre, Eigen::MatrixXd& p_ob, Eigen::MatrixXd& C) {
    C = Eigen::MatrixXd::Zero(3,6);
    C(0,0) = 1;
    C(1,1) = 1;
    C(2,2) = 1;
    p_ob = C * s_pre;
    // Can be extended to more complex observation relations
}

// Main Kalman filter process
void kf_filter(Eigen::MatrixXd& s_est, Eigen::MatrixXd& P_est, double dt, const Eigen::Vector3d& observation) {
    Eigen::MatrixXd Q = 1e-7 * Eigen::MatrixXd::Identity(6,6); // Process noise
    Eigen::MatrixXd R = 5e-5 * Eigen::MatrixXd::Identity(3,3); // Measurement noise

    Eigen::MatrixXd F, s_pre;
    motion_model(s_est, dt, s_pre, F);
    Eigen::MatrixXd PPre = F * P_est * F.transpose() + Q;

    Eigen::MatrixXd p_ob, C_ob;
    observation_model(s_pre, p_ob, C_ob);

    Eigen::MatrixXd innovation = observation - p_ob;
    Eigen::MatrixXd S = C_ob * PPre * C_ob.transpose() + R;
    Eigen::MatrixXd K = PPre * C_ob.transpose() * S.inverse();

    s_est = s_pre + K * innovation;
    P_est = (Eigen::MatrixXd::Identity(6,6) - K * C_ob) * PPre;
}


// Average rotation vectors
cv::Mat compute_rotation_avg(const std::vector<cv::Mat>& rvecs) {
    cv::Mat avg_rot = cv::Mat::zeros(3,3,CV_32FC1);
    for (const auto& rvec : rvecs) {
        cv::Mat rot_mat(3,3,CV_32FC1);
        cv::Rodrigues(rvec, rot_mat);
        avg_rot += rot_mat;
    }
    avg_rot /= static_cast<float>(rvecs.size());
    float det = cv::determinant(avg_rot.t());
    cv::Mat w, u, vt;
    cv::SVDecomp(avg_rot.t(), w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat result;
    if (det > 0) {
        result = vt.t() * u.t();
    } else {
        cv::Mat H = cv::Mat::eye(3,3,CV_32FC1);
        H.at<float>(2,2) = -1;
        result = vt.t() * H * u.t();
    }
    cv::Mat rvec_result;
    cv::Rodrigues(result, rvec_result);
    return rvec_result;
}


// Quaternion to Euler angles
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) {
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    Eigen::Vector3d euler;
    euler[0] = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    euler[1] = asin(2.0 * (qw * qy - qz * qx));
    euler[2] = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    return euler;
}

// Euler angles to rotation matrix
void euler_to_rotation(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation) {
    double cy = cos(euler[1]), sy = sin(euler[1]);
    double cp = cos(euler[2]), sp = sin(euler[2]);
    double cr = cos(euler[0]), sr = sin(euler[0]);
    rotation(0,0) = cy * cp;
    rotation(0,1) = sy * sr * cp - cr * sp;
    rotation(0,2) = sy * cr * cp + sr * sp;
    rotation(1,0) = cy * sp;
    rotation(1,1) = sy * sr * sp + cr * cp;
    rotation(1,2) = -sr * cp + cr * sy * sp;
    rotation(2,0) = -sy;
    rotation(2,1) = -sr * cy;
    rotation(2,2) = cr * cy;
}