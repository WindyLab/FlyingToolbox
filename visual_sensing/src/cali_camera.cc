#include <ctime>
#include <cstdlib>
#include <chrono>
#include <opencv2/opencv.hpp> 
//#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/aruco.hpp> 

#include <fstream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "pose_filter.h"

 #include <chrono>

using sysclock_t = std::chrono::system_clock;

std::string CurrentDate() {
    std::time_t now = sysclock_t::to_time_t(sysclock_t::now());
    char buf[32] = { 0 };
    std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", std::localtime(&now));
    return std::string(buf);
}

void compute_camera_via_board(const cv::Mat& image,cv::Mat& r_mat,cv::Mat& t_vec,
bool display = false) {
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = new cv::aruco::DetectorParameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
   // cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_32FC1);
    cv::Mat distCoeffs(5,1,CV_32FC1);
    cameraMatrix.at<float>(0,0) = 607.019;
    cameraMatrix.at<float>(1,1) = 607.019;
    cameraMatrix.at<float>(0,2) = 315.875;
    cameraMatrix.at<float>(1,2) = 248.763;
    distCoeffs.at<float>(0) = 0.182874;
    distCoeffs.at<float>(1) = -0.56215;
    distCoeffs.at<float>(2) = -0.000804634;
    distCoeffs.at<float>(3) = -0.00028305;
    distCoeffs.at<float>(4) = 0.502297;


    cv::Mat objPoints(4, 1, CV_32FC3);
    float markerLength = 0.07;
    int origin_id = 31;
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f + 0.118, markerLength / 2.f + 0.05, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f + 0.118, markerLength / 2.f+ 0.05, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f + 0.118, -markerLength / 2.f+ 0.05, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f + 0.118, -markerLength / 2.f+ 0.05, 0);

 //   cv::cvtColor(image,image,cv::COLOR_RGB2BGR);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners,rejectedCandidates;

        cv::aruco::detectMarkers(image, 
    dictionary, 
    corners, 
    ids, detectorParams, rejectedCandidates);

 //   detector.detectMarkers(image, corners, ids);
 //    printf("nMarkers: %d\n",int(ids.size()));

    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        int nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        std::vector<int>::iterator origin_id_iter = std::find(ids.begin(),ids.end(),origin_id);
        int origin_index = -1;
        if(origin_id_iter != ids.end()) {
            origin_index = std::distance(ids.begin(),origin_id_iter);
        } else {
            std::cout << "can not find origin id tag!\n";
            return ;
        }
        std::vector<cv::Mat> rvec_mat;
        // Calculate pose for each marker
        for (int i = 0; i < nMarkers; i++) {
            cv::solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            cv::Mat current = cv::Mat( rvecs[i] );
            rvec_mat.push_back(current);
           // std::cout << i << " " << tvecs.at(i) << std::endl;
           // std::cout << '\n';
        }
        t_vec = cv::Mat(tvecs.at(origin_index));
        cv::Mat res = compute_rotation_avg(rvec_mat);
        cv::Rodrigues(res,r_mat);
//        std::cout << ids[origin_index] << " " << origin_index << std::endl;
//      std::cout << "result rotation:" << res << std::endl;
//      std::cout << "resultes translation:" << t_vec << std::endl;

        if(display) {
        // Draw axis for each marker
            for(unsigned int i = 0; i < ids.size(); i++) {
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.07);
            }
        }
    }
    if (display) {
        // Update the window with new data
        cv::imshow("window", image);
        cv::waitKey();
    }
}


class RealSenseMonitor {
    public:
    

  RealSenseMonitor(ros::NodeHandle& nh): nh_(nh),it_(nh_) { 
    // init calibration parameters
    init_pts3d();
    detectorParams_ = new cv::aruco::DetectorParameters;
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cameraMatrix_ = cv::Mat::eye(3,3,CV_32FC1);
    distCoeffs_ = cv::Mat(5,1,CV_32FC1);
    cameraMatrix_.at<float>(0,0) = 607.019;
    cameraMatrix_.at<float>(1,1) = 607.019;
    cameraMatrix_.at<float>(0,2) = 315.875;
    cameraMatrix_.at<float>(1,2) = 248.763;
    distCoeffs_.at<float>(0) = 0.182874;
    distCoeffs_.at<float>(1) = -0.56215;
    distCoeffs_.at<float>(2) = -0.000804634;
    distCoeffs_.at<float>(3) = -0.00028305;
    distCoeffs_.at<float>(4) = 0.502297;
    image_sub_ = it_.subscribe("/camera/realsense", 1, &RealSenseMonitor::imageCb, this);
  }

    ~RealSenseMonitor(){

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    float t = msg->header.stamp.toSec();
    img_ = (cv_ptr->image);
  }


  void monitor(cv::Mat& r_mat,cv::Mat& tvec,
  const tf::Transform& trans_world_viconobj_cam,
  const tf::Transform& trans_world_droneup
  ) {
        if (img_.empty()) {
            return ;
        }
        cv::Mat image = img_.clone();
        
        cv::cvtColor(image,image,cv::COLOR_RGB2BGR);

        cv::Mat rvec = cv::Mat::zeros(3,1,CV_32FC1);
        ros::Time time_stamp;
        int num_of_detection;
        bool draw_frames = false;
        solve_pose_all(image,rvec,tvec,time_stamp,num_of_detection,draw_frames);
        std::cout << "tag in camera:" << rvec << " " << tvec << " number of detection:" <<
        num_of_detection << std::endl;
        cv::Rodrigues( rvec,r_mat);
        r_mat.convertTo(r_mat,CV_32FC1);
        tvec.convertTo(tvec,CV_64FC1);
  }



  bool solve_pose_all(const cv::Mat& image, cv::Mat& rvec,cv::Mat& tvec,ros::Time& time_stamp,
  int& num_of_detection,bool draw_frames = false) {
        if(image.empty()) {
            printf("img is empty!\n");
            return false;
        }
        cv::Mat filtered = image.clone();
        cv::Mat filtered_gray; 
        cv::cvtColor(filtered,filtered_gray,cv::COLOR_BGR2GRAY);

        // cv::GaussianBlur(filtered_gray,filtered_gray,cv::Size(3,3),1);
        time_stamp = ros::Time::now();
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners,rejectedCandidates;
        cv::aruco::detectMarkers(filtered_gray, dictionary_, corners, ids,
        detectorParams_,rejectedCandidates);
        std::vector<cv::Mat> r_tag_vec;
      //  printf("number of detected markers: %lu\n",ids.size());
        if (ids.size() > 0) {
            int nMarkers = static_cast<int>(corners.size());
            std::vector<cv::Point2f> pts2d;
            std::vector<cv::Point3f> pts3d;
            std::map<int,std::vector<cv::Point2f>> pts2d_map;
            for (int i = 0; i < nMarkers; i++) {
               // printf("current id %d\n",ids[i]);
                if (ids[i] > 16) {
                    continue;
                }
                for(size_t k  = 0; k < 4; k++) {
                    pts2d.push_back(corners.at(i)[k]);
                }
                auto map_iter = pts3d_map_.find(ids[i]);
                for(size_t k = 0; k < map_iter->second.size(); k++) {
                    pts3d.push_back(map_iter->second[k]);
                }
            }
           // std::cout << "pts 2d size:" << pts2d.size() << std::endl;
           // std::cout << "pts 3d size:" << pts3d.size() << std::endl;
            if (pts2d.empty()) {
                printf("no valid points detected!\n");
                return false;
            }
            cv::solvePnP(pts3d, pts2d,cameraMatrix_, distCoeffs_, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);
        }
        tvec = tvec / 1000;
        num_of_detection = static_cast<int>(ids.size());
        if(draw_frames) {
            if (ids.size() > 0) {
                cv::aruco::drawAxis(filtered, cameraMatrix_, distCoeffs_, rvec, tvec, 0.03);
                cv::aruco::drawDetectedMarkers(filtered, corners, ids);
            }
            cv::imshow(window_name_, filtered);
            char key = cv::waitKey(1);
            if (key == ' ') {
                static int cnt = 0;
                printf("save image!\n");
                cv::imwrite("image" + std::to_string(cnt++) + ".png",filtered);
            }
        }
        if (ids.size() > 0) {
            return true;
        } else {
            return false;
        }
  }


private:
    void init_pts3d();
    const std::string window_name_ = "Display Image";
    std::map<int,std::vector<cv::Point3f>> pts3d_map_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat img_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

};

void RealSenseMonitor::init_pts3d() {
    std::map<int,std::vector<cv::Point3f>> pts3d_map =
        {   {0,
                {
                    cv::Point3f(-14.5, 14.5,   0),
                    cv::Point3f(14.5,  14.5,   0),
                    cv::Point3f(14.5,  -14.5,   0),
                    cv::Point3f(-14.5, -14.5,   0)

                }
            },
            {1,
                {
                    cv::Point3f(118.0, 14.5,    0),
                    cv::Point3f(147.0, 14.5,    0),
                    cv::Point3f(147.0, -14.5,   0),
                    cv::Point3f(118.0, -14.5,   0)

                }
            },
            {2,
                {
                    cv::Point3f(-14.5, -118.0,   0),
                    cv::Point3f(14.5,  -118.0,   0),
                    cv::Point3f(14.5,  -147.0,   0),
                    cv::Point3f(-14.5, -147.0,   0)
                }
            },
            {3,
                {
                    cv::Point3f(-147.0, 14.5,   0),
                    cv::Point3f(-118.0, 14.5,   0),
                    cv::Point3f(-118.0, -14.5,  0),
                    cv::Point3f(-147.0, -14.5,  0)
                }
            },
            {4,
                {
                    cv::Point3f(-14.5, 147.0,  0),
                    cv::Point3f(14.5,  147.0,   0),
                    cv::Point3f(14.5,  118.0,   0),
                    cv::Point3f(-14.5, 118.0,  0)
                }
            },
            {5,
                {
                    cv::Point3f(60.75, 175.0,  0),
                    cv::Point3f(86.75, 175.0,   0),
                    cv::Point3f(86.75, 149.0,   0),
                    cv::Point3f(60.75, 149.0,  0)
                }
            },
            {6,
                {
                    cv::Point3f(149.0, 86.75,  0),
                    cv::Point3f(175.0, 86.75,   0),
                    cv::Point3f(175.0, 60.75,   0),
                    cv::Point3f(149.0, 60.75,  0)
                }
            },
            {7,
                {
                    cv::Point3f(149.0, -60.75,  0),
                    cv::Point3f(175.0, -60.75,   0),
                    cv::Point3f(175.0, -86.75,   0),
                    cv::Point3f(149.0, -86.75,  0)
                }
            },
            {8,
                {
                    cv::Point3f(60.75, -149.0,  0),
                    cv::Point3f(86.75, -149.0,   0),
                    cv::Point3f(86.75, -175.0,   0),
                    cv::Point3f(60.75, -175.0,  0)
                }
            },
            {9,
                {
                    cv::Point3f(-86.75, -149.0,  0),
                    cv::Point3f(-60.75, -149.0,   0),
                    cv::Point3f(-60.75, -175.0,   0),
                    cv::Point3f(-86.75, -175.0,  0)
                }
            }, //10: [[-175.0, -60.75], [-149.0, -60.75], [-149.0, -86.75], [-175.0, -86.75]]
            {10,
                {
                    cv::Point3f(-175.0, -60.75,  0),
                    cv::Point3f(-149.0, -60.75,   0),
                    cv::Point3f(-149.0, -86.75,   0),
                    cv::Point3f(-175.0, -86.75,  0)
                }
            },// 11: [[-175.0, 86.75], [-149.0, 86.75], [-149.0, 60.75], [-175.0, 60.75]],
            //12: [[-86.75, 175.0], [-60.75, 175.0], [-60.75, 149.0], [-86.75, 149.0]]}
            {11,
                {
                    cv::Point3f(-175.0, 86.75,  0),
                    cv::Point3f(-149.0, 86.75,   0),
                    cv::Point3f(-149.0, 60.75,   0),
                    cv::Point3f(-175.0, 60.75,  0)
                }
            },
            {12,
                {
                    cv::Point3f(-86.75, 175.0,  0),
                    cv::Point3f(-60.75, 175.0,   0),
                    cv::Point3f(-60.75, 149.0,   0),
                    cv::Point3f(-86.75, 149.0,  0)
                }
            }, 
            {13,
                {
                    cv::Point3f(126.5, 146.5,  0),
                    cv::Point3f(146.5, 146.5,   0),
                    cv::Point3f(146.5, 126.5,   0),
                    cv::Point3f(126.5, 126.5,  0)
                }
            },
            {14,
                {
                    cv::Point3f(126.5, -126.5,  0),
                    cv::Point3f(146.5, -126.5,   0),
                    cv::Point3f(146.5, -146.5,   0),
                    cv::Point3f(126.5, -146.5,  0)
                }
            },
            {15,
                {
                    cv::Point3f(-146.5, -126.5,  0),
                    cv::Point3f(-126.5, -126.5,   0),
                    cv::Point3f(-126.5, -146.5,   0),
                    cv::Point3f(-146.5, -146.5,  0)
                }
            },
            {16,
                {
                    cv::Point3f(-146.5, 146.5,  0),
                    cv::Point3f(-126.5, 146.5,   0),
                    cv::Point3f(-126.5, 126.5,   0),
                    cv::Point3f(-146.5, 126.5,  0)
                }
            }
        };
        pts3d_map_ = pts3d_map;
  }
  

int main(int argc, char * argv[]) {
    //init ros
    ros::init(argc, argv, "cali_camera_vicon");
    ros::NodeHandle nh;

    //listen to vicon transform
    tf::TransformListener* tfListener= new tf::TransformListener;
    ros::Rate rate(200);
    static int cnt = 0;
    RealSenseMonitor rs_monitor(nh);
    bool read_precalibrated = false;

    while(nh.ok()) {
        ros::spinOnce();
        bool tf_ok = true;
        tf::StampedTransform trans_world_viconobj; 
        tf::StampedTransform trans_world_droneup;  //drone's coordinate Turing
        tf::StampedTransform trans_droneup_viconcam_obj;
        tf::StampedTransform trans_viconworld_viconboard;

        try {
          tfListener->lookupTransform("/vicon/world", "/vicon/Cali_ruler/Cali_ruler", ros::Time(0), trans_viconworld_viconboard);
        // for Turing
          tfListener->lookupTransform("/vicon/world", "/vicon/realsense/realsense", ros::Time(0), trans_world_viconobj);
          tfListener->lookupTransform("/vicon/world", "/vicon/Turing/Turing", ros::Time(0), trans_world_droneup);
          tfListener->lookupTransform("vicon/Turing/Turing", "/vicon/realsense/realsense", ros::Time(0), trans_droneup_viconcam_obj);
                } catch(tf::TransformException ex) {
            ROS_ERROR("-------> %s", ex.what());
            tf_ok = false;
        }
        if(tf_ok) {
            cv::Mat r_mat(3,3,CV_32FC1);
            cv::Mat t_vec(3,1,CV_32FC1);
            rs_monitor.monitor(r_mat,t_vec,trans_world_viconobj,trans_world_droneup);

        tf::Matrix3x3 tf_rot(
            r_mat.at<float>(0, 0), r_mat.at<float>(0, 1), r_mat.at<float>(0, 2),
            r_mat.at<float>(1, 0), r_mat.at<float>(1, 1), r_mat.at<float>(1, 2),
            r_mat.at<float>(2, 0), r_mat.at<float>(2, 1), r_mat.at<float>(2, 2));
        tf::Transform trans_cam_board(tf_rot, tf::Vector3(t_vec.at<double>(0),t_vec.at<double>(1),t_vec.at<double>(2)));

        auto trans_camera2hand = trans_world_droneup.inverse() * trans_viconworld_viconboard * trans_cam_board.inverse();

        tf::Transform trans_cam_viconworld = trans_cam_board * trans_viconworld_viconboard.inverse();
        auto trans_viconobj_world = trans_world_viconobj.inverse();
        auto trans_viconobj_cam = trans_viconobj_world * trans_cam_viconworld.inverse();
        auto trans_drone_cam = trans_droneup_viconcam_obj * trans_viconobj_cam;
        std::cout << trans_viconobj_cam.getOrigin().x() << " " << trans_viconobj_cam.getOrigin().y() << " " << 
           trans_viconobj_cam.getOrigin().z() << std::endl;

        std::cout << "trans_drone_cam:" << trans_drone_cam.getOrigin().x() << " " << trans_drone_cam.getOrigin().y() << " " << 
            trans_drone_cam.getOrigin().z() << std::endl; 
        std::cout << "trans_camera2hand:" << trans_camera2hand.getOrigin().x() << " " << trans_camera2hand.getOrigin().y() << " " << 
            trans_camera2hand.getOrigin().z() << std::endl; 
         
        if (cnt ++ == 100) {
            std::ofstream fs("calibration_trans_drone_cam_" + CurrentDate() + ".txt");
            // qx qy qz qw x y z
            fs << trans_drone_cam.getRotation().x() << " " << 
                trans_drone_cam.getRotation().y() << " " << 
            trans_drone_cam.getRotation().z() << " " << trans_drone_cam.getRotation().w() << " "
            << trans_drone_cam.getOrigin().x() << " " << trans_drone_cam.getOrigin().y() << " "
            << trans_drone_cam.getOrigin().z() << std::endl;
            Eigen::Quaterniond q_;
            q_.x() = trans_drone_cam.getRotation().x();
            q_.y() = trans_drone_cam.getRotation().y();
            q_.z() = trans_drone_cam.getRotation().z();
            q_.w() = trans_drone_cam.getRotation().w();

            fs << quaternion_to_euler(q_)(0) * 180.f / M_PI << " " << quaternion_to_euler(q_)(1)* 180.f / M_PI << " " << quaternion_to_euler(q_)(2)* 180.f / M_PI << std::endl;
            fs.close();
            printf("Extrinsics saved!\n");
        }
        }
        rate.sleep();
     
    }
    delete tfListener;
    return EXIT_SUCCESS;

}