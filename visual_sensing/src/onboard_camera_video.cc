#include <ctime>
#include <iomanip> 
#include <sstream> 
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp> 
//#include <opencv2/objdetect/aruco_detector.hpp> //opencv 4.7.0
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>  // For VideoWriter

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "pose_filter.h"
#include "CLI11.hpp"
#include "visual_sensing/toolbox_array.h"
#include "visual_sensing/toolbox.h"
#include "log4z.h"

class TicToc {
  public:
    TicToc() {
        tic();
    }
    void tic() {
        start = std::chrono::system_clock::now();
    }
    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

class ImageConverter
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  int mode_;

  ImageConverter(ros::NodeHandle& nh,
                 std::map<int,cv::Mat>& rot_map,
                 std::map<int,cv::Mat>& pos_map)
    : nh_(nh),it_(nh_) {
    // Subscribe image
    image_sub_ = it_.subscribe("/camera/realsense", 1, &ImageConverter::imageCb, this);

    //============================================================

    // Iinit aruco related values
    detectorParams_ = new cv::aruco::DetectorParameters;
    detectorParams_->maxMarkerPerimeterRate = 5.0;
    detectorParams_->polygonalApproxAccuracyRate = 0.05;
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cameraMatrix_ = cv::Mat::eye(3,3,CV_32FC1);
    cameraMatrix_.at<float>(0,0) = 607.019; //Likai 611
    cameraMatrix_.at<float>(1,1) = 607.019; //Likai 611
    cameraMatrix_.at<float>(0,2) = 315.875; //Likai 315.875
    cameraMatrix_.at<float>(1,2) = 248.763; //Likai 248.763
    distCoeffs_ = cv::Mat(5,1,CV_32FC1);
    distCoeffs_.at<float>(0) = 0.182874;
    distCoeffs_.at<float>(1) = -0.56215;
    distCoeffs_.at<float>(2) = -0.000804634;
    distCoeffs_.at<float>(3) = -0.00028305;
    distCoeffs_.at<float>(4) = 0.502297;
    float markerLength = 0.029;
    objPoints_ = cv::Mat(4, 1, CV_32FC3);
    objPoints_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    rot_map_ = rot_map;
    pos_map_ = pos_map;
    mode_ = 0;
  }

  ~ImageConverter(){

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


  void init_pts3d_toolbox() {
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
            },
            {10,
                {
                    cv::Point3f(-175.0, -60.75,  0),
                    cv::Point3f(-149.0, -60.75,   0),
                    cv::Point3f(-149.0, -86.75,   0),
                    cv::Point3f(-175.0, -86.75,  0)
                }
            },
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

  bool solve_pose_all(cv::Mat& rvec,cv::Mat& tvec,ros::Time& time_stamp,
  int& num_of_detection,bool draw_frames = true) {
        if(img_.empty()) {
            printf("img is empty!\n");
            return false;
        }
        cv::Mat filtered = img_.clone();
        cv::Mat filtered_gray; 
        cv::cvtColor(filtered,filtered_gray,cv::COLOR_BGR2GRAY);

        time_stamp = ros::Time::now();
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners,rejectedCandidates;
        cv::aruco::detectMarkers(filtered_gray, dictionary_, corners, ids,
        detectorParams_,rejectedCandidates);
        std::vector<cv::Mat> r_tag_vec;
        bool flag_solved = false;
        int size_of_useful_tag = 0;
        ROS_INFO("ids.size()=%d",ids.size());
        if (ids.size() > 0) {
            int nMarkers = static_cast<int>(corners.size());
            std::vector<cv::Point2f> pts2d;
            std::vector<cv::Point3f> pts3d;
            std::map<int,std::vector<cv::Point2f>> pts2d_map;
            for (int i = 0; i < nMarkers; i++) {

                if (ids[i] > 16) {
                    mode_ = 1; // detect target
                } else if (ids[i] <= 16) {
                    mode_ = 0;  //detect tool
                    for(size_t k  = 0; k < 4; k++) {
                    pts2d.push_back(corners.at(i)[k]);
                    }

                    auto map_iter = pts3d_map_.find(ids[i]);
                    ROS_INFO("map_iter=%d",map_iter);
                    for(size_t k = 0; k < map_iter->second.size(); k++) {
                        pts3d.push_back(map_iter->second[k]);
                        ROS_INFO("second.size()=%d",map_iter->second.size());
                        ROS_INFO("k=%d",k);
                    }
                }
            }
            size_of_useful_tag = static_cast<int>(pts2d.size())/4;
            if (pts2d.empty()) {
                printf("no valid points detected!\n");
                // return false;
            }
            else
            {
                cv::solvePnP(pts3d, pts2d,cameraMatrix_, distCoeffs_, rvec, tvec,false,cv::SOLVEPNP_ITERATIVE);
                flag_solved = true;
            }
        }
        tvec = tvec / 1000;
        num_of_detection = size_of_useful_tag;
        if(draw_frames) {
            if (ids.size() > 0) {
                cv::aruco::drawAxis(filtered, cameraMatrix_, distCoeffs_, rvec, tvec, 0.03);
                cv::aruco::drawDetectedMarkers(filtered, corners, ids);
            }

            printf("save image!\n");
            // cv::imwrite("image.png",filtered);

            cv_bridge::CvImagePtr cv_ptr_aft = boost::make_shared<cv_bridge::CvImage>();

            cv_ptr_aft->header.stamp = ros::Time::now();   
            cv_ptr_aft->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr_aft->image = filtered; 

            if (!is_initialized_aft_)
            {
                int width = cv_ptr_aft->image.cols;
                int height = cv_ptr_aft->image.rows;

                std::string dateTimeString = getCurrentDateTimeString();
                std::string videoPath = "/home/nuc/waypoint_ws/logs/output_video_aft_" + dateTimeString + ".mp4";

                video_writer_aft_.open(videoPath, cv::VideoWriter::fourcc('H', '2', '6', '4'), 60, cv::Size(width, height));
                is_initialized_aft_ = true;
            }

            if (video_writer_aft_.isOpened())
            {
            video_writer_aft_.write(cv_ptr_aft->image);
            }

        }
        if (ids.size() > 0 && flag_solved) {
            return true;
        } else {
            return false;
        }
  }


  std::string getCurrentDateTimeString()
  {
      std::time_t now = std::time(nullptr);

      std::tm* now_tm = std::localtime(&now);
      std::stringstream ss;
      ss << std::put_time(now_tm, "%m-%d_%H-%M"); 

      return ss.str();
  }

  private:
    cv::Mat img_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Mat objPoints_;
    std::map<int,cv::Mat> rot_map_;
    std::map<int,cv::Mat> pos_map_;
    const std::string window_name_ = "VIEW";
    std::map<int,std::vector<cv::Point3f>> pts3d_map_;

    bool is_initialized_aft_ = false;
    cv::VideoWriter video_writer_aft_;
};


#if 1
int main(int argc, char * argv[]) try
{
    CLI::App app{"aerial toolbox"};
    bool draw_frames = true;
    int target_id = 0;
    app.add_option("-f", draw_frames, "Draw frames on the images or not");
    app.add_option("-i", target_id, "target ID");
    char *buffer;
    if((buffer = getcwd(NULL, 0)) == NULL)
    {
        printf("getcwd error\n");
    }
    else
    {
        printf("%s\n", buffer);
        free(buffer);
    }
    zsummer::log4z::ILog4zManager::getRef().start();  
    CLI11_PARSE(app, argc, argv);

    //Init all markers
    std::map<int,cv::Mat> rot_map;
    std::map<int,cv::Mat> pos_map;

    for(auto i = rot_map.begin(); i != rot_map.end(); i++) {
        std::cout << i->first << " " << i->second << std::endl;
    }

    //================Set target point=================
    // toolbox_points 0-3, toolbox
    // toolbox_points 4-8, target
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> toolbox_points;
    toolbox_points.resize(8);
    toolbox_points[0] = Eigen::Vector3d(0.0735,0.0735,0);
    toolbox_points[1] = Eigen::Vector3d(0.0735,-0.0735,0);  
    toolbox_points[2] = Eigen::Vector3d(-0.0735,-0.0735,0);
    toolbox_points[3] = Eigen::Vector3d(-0.0735,0.0735,0);

    toolbox_points[4] = Eigen::Vector3d::Zero();
    toolbox_points[5] = Eigen::Vector3d::Zero();
    toolbox_points[6] = Eigen::Vector3d::Zero();
    toolbox_points[7] = Eigen::Vector3d::Zero();

    //=================================================
    // Init ros 
    ros::init(argc, argv, "onboard_camera_video");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ImageConverter ic(nh,rot_map,pos_map);

    double publish_frequency;

    private_nh.param<double>("publish_frequency", publish_frequency, 50);


    std::string tf_prefix = "/vicon/realsense/realsense";
    ros::Rate rate(publish_frequency);
    tf::TransformListener*  tfListener= new tf::TransformListener;

    ic.init_pts3d_toolbox();

    int num_of_detection = 0;

    while(ros::ok()) {
        ros::spinOnce();

        bool tf_ok = true;
        cv::Mat rvec = cv::Mat::zeros(3,1,CV_32FC1);
        cv::Mat r_mat = cv::Mat::eye(3,3,CV_32FC1);
        cv::Mat tvec = cv::Mat::zeros(3,1,CV_32FC1);
        ros::Time time_stamp;
        try {
            tf_ok = ic.solve_pose_all(rvec,tvec,time_stamp,num_of_detection,draw_frames);
        } catch(tf::TransformException ex) {
            ROS_ERROR("-------> %s", ex.what());
            tf_ok = false;
        }
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
#endif