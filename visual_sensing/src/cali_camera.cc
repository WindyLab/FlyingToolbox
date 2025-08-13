#include <chrono>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "pose_filter.h"

std::string GetTimeString() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", std::localtime(&now_c));
    return std::string(buf);
}

void EstimateCameraPose(const cv::Mat& img, cv::Mat& rotMat, cv::Mat& transVec, bool show = false) {
    auto params = cv::aruco::DetectorParameters::create();
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Mat camMat = cv::Mat::eye(3,3,CV_32FC1);
    cv::Mat dist(5,1,CV_32FC1);
    camMat.at<float>(0,0) = 607.019f;
    camMat.at<float>(1,1) = 607.019f;
    camMat.at<float>(0,2) = 315.875f;
    camMat.at<float>(1,2) = 248.763f;
    dist.at<float>(0) = 0.182874f;
    dist.at<float>(1) = -0.56215f;
    dist.at<float>(2) = -0.000804634f;
    dist.at<float>(3) = -0.00028305f;
    dist.at<float>(4) = 0.502297f;

    float markerLen = 0.07f;
    int ref_id = 31;
    cv::Mat objPts(4, 1, CV_32FC3);
    objPts.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLen/2+0.118, markerLen/2+0.05, 0);
    objPts.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLen/2+0.118, markerLen/2+0.05, 0);
    objPts.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLen/2+0.118, -markerLen/2+0.05, 0);
    objPts.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLen/2+0.118, -markerLen/2+0.05, 0);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(img, dict, corners, ids, params, rejected);

    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(img, corners, ids);
        int n = corners.size();
        std::vector<cv::Vec3d> rvecs(n), tvecs(n);
        auto it = std::find(ids.begin(), ids.end(), ref_id);
        int idx = (it != ids.end()) ? std::distance(ids.begin(), it) : -1;
        if (idx < 0) {
            std::cout << "Reference marker not found!\n";
            return;
        }
        std::vector<cv::Mat> rvecMatList;
        for (int i = 0; i < n; ++i) {
            cv::solvePnP(objPts, corners[i], camMat, dist, rvecs[i], tvecs[i]);
            rvecMatList.push_back(cv::Mat(rvecs[i]));
        }
        transVec = cv::Mat(tvecs[idx]);
        cv::Mat avgRot = compute_rotation_avg(rvecMatList);
        cv::Rodrigues(avgRot, rotMat);

        if (show) {
            for (unsigned int i = 0; i < ids.size(); ++i) {
                cv::aruco::drawAxis(img, camMat, dist, rvecs[i], tvecs[i], 0.07);
            }
        }
    }
    if (show) {
        cv::imshow("window", img);
        cv::waitKey();
    }
}

class CameraPoseMonitor {
public:
    CameraPoseMonitor(ros::NodeHandle& nh) : nh_(nh), it_(nh_) {
        Setup3DPoints();
        params_ = cv::aruco::DetectorParameters::create();
        dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        camMat_ = cv::Mat::eye(3,3,CV_32FC1);
        dist_ = cv::Mat(5,1,CV_32FC1);
        camMat_.at<float>(0,0) = 607.019f;
        camMat_.at<float>(1,1) = 607.019f;
        camMat_.at<float>(0,2) = 315.875f;
        camMat_.at<float>(1,2) = 248.763f;
        dist_.at<float>(0) = 0.182874f;
        dist_.at<float>(1) = -0.56215f;
        dist_.at<float>(2) = -0.000804634f;
        dist_.at<float>(3) = -0.00028305f;
        dist_.at<float>(4) = 0.502297f;
        image_sub_ = it_.subscribe("/camera/realsense", 1, &CameraPoseMonitor::ImageCallback, this);
    }

    ~CameraPoseMonitor() {}

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void GetPose(cv::Mat& rotMat, cv::Mat& transVec,
                 const tf::Transform& world_cam,
                 const tf::Transform& world_drone) {
        if (img_.empty()) return;
        cv::Mat frame = img_.clone();
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        cv::Mat rvec = cv::Mat::zeros(3,1,CV_32FC1);
        ros::Time stamp;
        int detected = 0;
        bool draw = false;
        DetectPose(frame, rvec, transVec, stamp, detected, draw);
        std::cout << "Detected tag pose: " << rvec << " " << transVec << " count: " << detected << std::endl;
        cv::Rodrigues(rvec, rotMat);
        rotMat.convertTo(rotMat, CV_32FC1);
        transVec.convertTo(transVec, CV_64FC1);
    }

    bool DetectPose(const cv::Mat& frame, cv::Mat& rvec, cv::Mat& tvec, ros::Time& stamp,
                    int& detected, bool draw = false) {
        if (frame.empty()) {
            std::cout << "Frame is empty!\n";
            return false;
        }
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        stamp = ros::Time::now();
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(gray, dict_, corners, ids, params_, rejected);

        std::vector<cv::Point2f> pts2d;
        std::vector<cv::Point3f> pts3d;
        for (size_t i = 0; i < corners.size(); ++i) {
            if (ids[i] > 16) continue;
            pts2d.insert(pts2d.end(), corners[i].begin(), corners[i].end());
            auto it = pts3d_map_.find(ids[i]);
            pts3d.insert(pts3d.end(), it->second.begin(), it->second.end());
        }
        if (pts2d.empty()) {
            std::cout << "No valid points!\n";
            return false;
        }
        cv::solvePnP(pts3d, pts2d, camMat_, dist_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        tvec = tvec / 1000;
        detected = static_cast<int>(ids.size());
        if (draw && !ids.empty()) {
            cv::aruco::drawAxis(frame, camMat_, dist_, rvec, tvec, 0.03);
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            cv::imshow(window_name_, frame);
            char key = cv::waitKey(1);
            if (key == ' ') {
                static int cnt = 0;
                cv::imwrite("img_" + std::to_string(cnt++) + ".png", frame);
            }
        }
        return !ids.empty();
    }

private:
    void Setup3DPoints();
    const std::string window_name_ = "ImageDisplay";
    std::map<int, std::vector<cv::Point3f>> pts3d_map_;
    cv::Mat camMat_;
    cv::Mat dist_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Mat img_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};

void CameraPoseMonitor::Setup3DPoints() {
    std::map<int, std::vector<cv::Point3f>> pts = {
        {0, {cv::Point3f(-14.5, 14.5, 0), cv::Point3f(14.5, 14.5, 0), cv::Point3f(14.5, -14.5, 0), cv::Point3f(-14.5, -14.5, 0)}},
        {1, {cv::Point3f(118.0, 14.5, 0), cv::Point3f(147.0, 14.5, 0), cv::Point3f(147.0, -14.5, 0), cv::Point3f(118.0, -14.5, 0)}},
        {2, {cv::Point3f(-14.5, -118.0, 0), cv::Point3f(14.5, -118.0, 0), cv::Point3f(14.5, -147.0, 0), cv::Point3f(-14.5, -147.0, 0)}},
        {3, {cv::Point3f(-147.0, 14.5, 0), cv::Point3f(-118.0, 14.5, 0), cv::Point3f(-118.0, -14.5, 0), cv::Point3f(-147.0, -14.5, 0)}},
        {4, {cv::Point3f(-14.5, 147.0, 0), cv::Point3f(14.5, 147.0, 0), cv::Point3f(14.5, 118.0, 0), cv::Point3f(-14.5, 118.0, 0)}},
        {5, {cv::Point3f(60.75, 175.0, 0), cv::Point3f(86.75, 175.0, 0), cv::Point3f(86.75, 149.0, 0), cv::Point3f(60.75, 149.0, 0)}},
        {6, {cv::Point3f(149.0, 86.75, 0), cv::Point3f(175.0, 86.75, 0), cv::Point3f(175.0, 60.75, 0), cv::Point3f(149.0, 60.75, 0)}},
        {7, {cv::Point3f(149.0, -60.75, 0), cv::Point3f(175.0, -60.75, 0), cv::Point3f(175.0, -86.75, 0), cv::Point3f(149.0, -86.75, 0)}},
        {8, {cv::Point3f(60.75, -149.0, 0), cv::Point3f(86.75, -149.0, 0), cv::Point3f(86.75, -175.0, 0), cv::Point3f(60.75, -175.0, 0)}},
        {9, {cv::Point3f(-86.75, -149.0, 0), cv::Point3f(-60.75, -149.0, 0), cv::Point3f(-60.75, -175.0, 0), cv::Point3f(-86.75, -175.0, 0)}},
        {10, {cv::Point3f(-175.0, -60.75, 0), cv::Point3f(-149.0, -60.75, 0), cv::Point3f(-149.0, -86.75, 0), cv::Point3f(-175.0, -86.75, 0)}},
        {11, {cv::Point3f(-175.0, 86.75, 0), cv::Point3f(-149.0, 86.75, 0), cv::Point3f(-149.0, 60.75, 0), cv::Point3f(-175.0, 60.75, 0)}},
        {12, {cv::Point3f(-86.75, 175.0, 0), cv::Point3f(-60.75, 175.0, 0), cv::Point3f(-60.75, 149.0, 0), cv::Point3f(-86.75, 149.0, 0)}},
        {13, {cv::Point3f(126.5, 146.5, 0), cv::Point3f(146.5, 146.5, 0), cv::Point3f(146.5, 126.5, 0), cv::Point3f(126.5, 126.5, 0)}},
        {14, {cv::Point3f(126.5, -126.5, 0), cv::Point3f(146.5, -126.5, 0), cv::Point3f(146.5, -146.5, 0), cv::Point3f(126.5, -146.5, 0)}},
        {15, {cv::Point3f(-146.5, -126.5, 0), cv::Point3f(-126.5, -126.5, 0), cv::Point3f(-126.5, -146.5, 0), cv::Point3f(-146.5, -146.5, 0)}},
        {16, {cv::Point3f(-146.5, 146.5, 0), cv::Point3f(-126.5, 146.5, 0), cv::Point3f(-126.5, 126.5, 0), cv::Point3f(-146.5, 126.5, 0)}}
    };
    pts3d_map_ = pts;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "camera_pose_calib");
    ros::NodeHandle nh;
    tf::TransformListener* tfListener = new tf::TransformListener;
    ros::Rate loop_rate(200);
    int frame_count = 0;
    CameraPoseMonitor monitor(nh);

    while (nh.ok()) {
        ros::spinOnce();
        bool tf_ready = true;
        tf::StampedTransform world_board, world_cam, world_drone, drone_cam, board_world;
        try {
            tfListener->lookupTransform("/vicon/world", "/vicon/Cali_ruler/Cali_ruler", ros::Time(0), board_world);
            tfListener->lookupTransform("/vicon/world", "/vicon/realsense/realsense", ros::Time(0), world_cam);
            tfListener->lookupTransform("/vicon/world", "/vicon/Turing/Turing", ros::Time(0), world_drone);
            tfListener->lookupTransform("vicon/Turing/Turing", "/vicon/realsense/realsense", ros::Time(0), drone_cam);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("TF Exception: %s", ex.what());
            tf_ready = false;
            ros::shutdown();
        }
        if (tf_ready) {
            cv::Mat rotMat(3,3,CV_32FC1), transVec(3,1,CV_32FC1);
            monitor.GetPose(rotMat, transVec, world_cam, world_drone);

            tf::Matrix3x3 tf_rot(
                rotMat.at<float>(0,0), rotMat.at<float>(0,1), rotMat.at<float>(0,2),
                rotMat.at<float>(1,0), rotMat.at<float>(1,1), rotMat.at<float>(1,2),
                rotMat.at<float>(2,0), rotMat.at<float>(2,1), rotMat.at<float>(2,2));
            tf::Transform cam_board(tf_rot, tf::Vector3(transVec.at<double>(0), transVec.at<double>(1), transVec.at<double>(2)));

            auto cam2hand = world_drone.inverse() * board_world * cam_board.inverse();
            auto cam_vicon = cam_board * board_world.inverse();
            auto viconobj_world = world_cam.inverse();
            auto viconobj_cam = viconobj_world * cam_vicon.inverse();
            auto drone_cam_trans = drone_cam * viconobj_cam;

            std::cout << "viconobj_cam: " << viconobj_cam.getOrigin().x() << " " << viconobj_cam.getOrigin().y() << " " << viconobj_cam.getOrigin().z() << std::endl;
            std::cout << "drone_cam_trans: " << drone_cam_trans.getOrigin().x() << " " << drone_cam_trans.getOrigin().y() << " " << drone_cam_trans.getOrigin().z() << std::endl;
            std::cout << "cam2hand: " << cam2hand.getOrigin().x() << " " << cam2hand.getOrigin().y() << " " << cam2hand.getOrigin().z() << std::endl;

            if (++frame_count == 100) {
                std::ofstream ofs("calibration_trans_drone_cam_" + GetTimeString() + ".txt");
                ofs << drone_cam_trans.getRotation().x() << " "
                    << drone_cam_trans.getRotation().y() << " "
                    << drone_cam_trans.getRotation().z() << " "
                    << drone_cam_trans.getRotation().w() << " "
                    << drone_cam_trans.getOrigin().x() << " "
                    << drone_cam_trans.getOrigin().y() << " "
                    << drone_cam_trans.getOrigin().z() << std::endl;
                Eigen::Quaterniond q;
                q.x() = drone_cam_trans.getRotation().x();
                q.y() = drone_cam_trans.getRotation().y();
                q.z() = drone_cam_trans.getRotation().z();
                q.w() = drone_cam_trans.getRotation().w();
                ofs << quaternion_to_euler(q)(0) * 180.0 / M_PI << " "
                    << quaternion_to_euler(q)(1) * 180.0 / M_PI << " "
                    << quaternion_to_euler(q)(2) * 180.0 / M_PI << std::endl;
                ofs.close();
                std::cout << "Calibration saved!\n";
            }
        }
        loop_rate.sleep();
    }
    delete tfListener;
    return EXIT_SUCCESS;
}