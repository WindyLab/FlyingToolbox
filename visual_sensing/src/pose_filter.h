#include <eigen3/Eigen/Dense>

void kf_filter(
Eigen::MatrixXd& s_est,
Eigen::MatrixXd& P_est,
const double dt,
const Eigen::Vector3d& observation);

cv::Mat compute_rotation_avg(const std::vector<cv::Mat>& rvecs);
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
void euler_to_rotation(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation);