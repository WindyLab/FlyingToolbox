#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>

void motion_model(
    const Eigen::MatrixXd& s_est,
    const double dt,
    Eigen::MatrixXd& s_pre,
    Eigen::MatrixXd& F) {
    F = Eigen::MatrixXd::Identity(6,6);
   // std::cout << F << std::endl;
    F(0,3) = dt;
    F(1,4) = dt;
    F(2,5) = dt;
    s_pre = F * s_est;
}

void observation_model(
    const Eigen::MatrixXd& s_pre,
     Eigen::MatrixXd& p_ob,
     Eigen::MatrixXd& C
    ) {

    C = Eigen::MatrixXd::Zero(3,6);
    C(0,0) = 1;
    C(1,1) = 1;
    C(2,2) = 1;
    p_ob = C * s_pre;
}


void kf_filter(
Eigen::MatrixXd& s_est,
Eigen::MatrixXd& P_est,
const double dt,
const Eigen::Vector3d& observation) {
    Eigen::MatrixXd Q = 0.0000001*Eigen::MatrixXd::Identity(6,6);
    // Q = Q*Q;
    Eigen::MatrixXd R = 0.00005*Eigen::MatrixXd::Identity(3,3);
    // R(0,0) = ;
    // R(1,1) = 0.001;
    // R(2,2) = 0.001;
    // R = R*R;
    Eigen::MatrixXd F;
    Eigen::MatrixXd s_pre;

    motion_model(s_est,dt,s_pre,F);
    Eigen::MatrixXd PPre = F * P_est * F.transpose() + Q;

    Eigen::MatrixXd p_ob;
    Eigen::MatrixXd C_ob;
    observation_model(s_pre,p_ob,C_ob);
    Eigen::MatrixXd innov = observation - p_ob;
    Eigen::MatrixXd S = C_ob * PPre * C_ob.transpose() + R ;
    Eigen::MatrixXd A = S.inverse();
    Eigen::MatrixXd K = PPre * C_ob.transpose() * A;
    s_est = s_pre + K*innov;
    P_est = (Eigen::MatrixXd::Identity(6,6) - K*C_ob)*PPre;
}

void readTxt(const std::string& file,
             std::vector<std::vector<float>>& res) {
    std::ifstream infile; 
    infile.open(file.data());
    assert(infile.is_open());

    std::string s;
    res.clear();
    while(getline(infile,s)) {
       // std::cout << s << std::endl;
        std::stringstream ss(s);
        float x,y,z;
        ss >> x >> y >> z;
        //std::cout << x << " " << y << " " << z << std::endl;
        std::vector<float> pos(3,0);
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
        res.push_back(pos);
    }
    infile.close();
}

cv::Mat compute_rotation_avg(const std::vector<cv::Mat>& rvecs) {
    cv::Mat r_alg_avg = cv::Mat::zeros(3,3,CV_32FC1);
    for(size_t i = 0; i < rvecs.size(); i++) {
        cv::Mat r_mat_current(3,3,CV_32FC1);
        cv::Rodrigues(rvecs[i],r_mat_current);
        r_alg_avg += r_mat_current;
    }
    r_alg_avg /= rvecs.size();
    float det = cv::determinant(r_alg_avg.t());
    cv::Mat w,u,vt;
    cv::SVDecomp(r_alg_avg.t(), w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);  //A = u w vT
    cv::Mat res;
    if ( det > 0 ) {
        res = vt.t()*u.t();
    } else {
        cv::Mat H = cv::Mat::eye(3,3,CV_32FC1);
        H.at<float>(2,2) = -1;
        res = vt.t()*H*u.t();
    }
    cv::Mat rvec_res;
    cv::Rodrigues(res,rvec_res);
    return rvec_res;
}

int test() {
    std::vector<std::vector<float>> res;
    readTxt("data.txt",res);
    size_t len = res.size();
    std::cout << len << std::endl;

    //initialization
    Eigen::MatrixXd s_est(6,1);
    s_est << res[0][0],res[0][1],res[0][2],0,0,0;
    Eigen::MatrixXd P_est = Eigen::MatrixXd::Identity(6,6);
    for(int k = 0; k < 6; k++) {
        P_est(k,k) = 0.1;
    }

    //iteration
    std::ofstream f("data_out.txt");
    for(int k = 0; k < len; k++) {
        Eigen::Vector3d observation;
        observation(0) = res[k][0];
        observation(1) = res[k][1];
        observation(2) = res[k][2];
        double dt = 0.0167;
        kf_filter(s_est,P_est,dt,observation);
        f << s_est(0) << " " << s_est(1) << " " << s_est(2) << std::endl;
        // std::cout << s_est << std::endl;
        // std::cout << observation << std::endl;
        // std::cout << std::endl;
    }
    f.close();
    return 0;
}

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

void euler_to_rotation(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation)
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
