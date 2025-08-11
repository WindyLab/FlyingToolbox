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
  image_transport::Publisher image_pub_;
  ros::Subscriber att_sub_;
  ros::Subscriber pos_sub_;
  int mode_;

  ImageConverter(ros::NodeHandle& nh,
                 std::map<int,cv::Mat>& rot_map,
                 std::map<int,cv::Mat>& pos_map)
    : nh_(nh),it_(nh_) {
    // Subscribe image
    image_sub_ = it_.subscribe("/camera/realsense", 1, &ImageConverter::imageCb, this);

    //=============Subscribe drone attitude and position=========
    att_sub_ = nh_.subscribe("/mavros/imu/data", 10, &ImageConverter::attCb,this);
    pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &ImageConverter::posCb,this);
    //============================================================

    // Iinit aruco related values
    detectorParams_ = new cv::aruco::DetectorParameters;
    detectorParams_->maxMarkerPerimeterRate = 5.0;
    detectorParams_->polygonalApproxAccuracyRate = 0.05;
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
    

    float markerLength = 0.029;
    objPoints_ = cv::Mat(4, 1, CV_32FC3);
    objPoints_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    rot_map_ = rot_map;
    pos_map_ = pos_map;
    // mode 0 -> localize toolbox
    // mode 1 -> localize target
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

  void attCb(const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, 
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z);
    rot_ = q_fcu.toRotationMatrix();
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    euler_fcu[1] = - euler_fcu[1];
    euler_fcu[2] = - euler_fcu[2];
    euler_to_rotation(euler_fcu,rotation_matrix_ned_);
  }

  void posCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    translation_ = Eigen::Vector3d(msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z);
  }

  void body2world(Eigen::Vector3d& target_position,const tf::StampedTransform& turing, Eigen::Vector3d& p_in_viconworld) {
    Eigen::Quaterniond q = Eigen::Quaterniond(turing.getRotation().w(), 
                    turing.getRotation().x(),
                    turing.getRotation().y(),
                    turing.getRotation().z());

    Eigen::Vector3d p(turing.getOrigin().x(),turing.getOrigin().y(),turing.getOrigin().z());
    Eigen::Matrix3d q_rot = q.toRotationMatrix();
    p_in_viconworld = q_rot * target_position + p;
  }

  void world2body(Eigen::Vector3d& p_in_world,
                  Eigen::Vector3d& p_in_body) {
    Eigen::Matrix3d delta_drone = Eigen::Matrix3d::Identity(3,3);
    delta_drone(1,1) = -1;
    delta_drone(2,2) = -1;
    p_in_body = rotation_matrix_ned_.transpose() * (delta_drone * (p_in_world - translation_));
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
  int& num_of_detection,bool draw_frames = false) {
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
                    for(size_t k = 0; k < map_iter->second.size(); k++) {
                        pts3d.push_back(map_iter->second[k]);
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

  void get_rotation_matrix(Eigen::Matrix3d& Rotation_body_2_world)
  {
    Rotation_body_2_world = rotation_matrix_ned_;
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
    tf::Transform body_in_viconworld_;
    Eigen::Matrix3d rot_;
    Eigen::Matrix3d rotation_matrix_ned_;
    Eigen::Vector3d translation_;
    std::map<int,std::vector<cv::Point3f>> pts3d_map_;

    bool is_initialized_aft_ = false;
    cv::VideoWriter video_writer_aft_;
};

void read_tag_extrinsics(
    const std::string& file,
    std::map<int,cv::Mat>& rot_map,
    std::map<int,cv::Mat>& pos_map) {
    std::ifstream ifs(file);
    if(!ifs.good()) {
        std::cerr << "ifstream open file error!\n";
        return;
    }
    std::string line;
    std::vector<std::string> lines;
    std::cout << "start to get line\n";
    while(getline(ifs, line)) {
        std::stringstream ss;
        ss << line;
        int tag_id = 0;
        cv::Mat rvec(3,1,CV_32FC1);
        cv::Mat tvec(3,1,CV_32FC1);

        ss >> tag_id >> rvec.at<float>(0) >> rvec.at<float>(1) >> rvec.at<float>(2) 
        >> tvec.at<float>(0) >> tvec.at<float>(1) >> tvec.at<float>(2);
        rot_map.insert(std::make_pair(tag_id,rvec));
        pos_map.insert(std::make_pair(tag_id,tvec));
        //std::cout << line << std::endl;
        std::cout << tag_id << " " << rvec << " " << tvec << std::endl;
    }
    ifs.close();
}

void read_camera_in_drone(const std::string& file, tf::Transform& trans_drone_cam) {
    std::ifstream ifs(file);
    if(!ifs.good()) {
        std::cerr << "ifstream open file error!\n";
        return;
    }
    std::string line;
    std::vector<std::string> lines;
    int tag_id = 0;
    while(getline(ifs, line)) {
        std::stringstream ss;
        ss << line;
        float q_data[4] = {0};// x y z w
        float xyz[3] = {0};
        ss  >> q_data[0] >>  q_data[1] >>  q_data[2] >>  q_data[3] >> 
        xyz[0] >> xyz[1] >> xyz[2];
        tf::Quaternion q(q_data[0],q_data[1],q_data[2],q_data[3]);
        tf::Vector3 trans(xyz[0],xyz[1],xyz[2]);
        std::cout << trans.x() << " " << trans.y() << " " << trans.z() << std::endl;
        trans_drone_cam = tf::Transform(q, trans);
    }
    ifs.close();
}

tf::Vector3 compute_target_in_vicon(const cv::Mat& rvec, const cv::Mat& tvec,
const tf::Transform& trans_drone_cam,
const tf::StampedTransform& trans_world_viconobj) {
    cv::Mat r_mat(3,3,CV_64FC1);
    cv::Rodrigues(rvec, r_mat);

    tf::Matrix3x3 rot_cam_tool(r_mat.at<float>(0, 0), r_mat.at<float>(0, 1), r_mat.at<float>(0, 2),
                        r_mat.at<float>(1, 0), r_mat.at<float>(1, 1), r_mat.at<float>(1, 2),
                        r_mat.at<float>(2, 0), r_mat.at<float>(2, 1), r_mat.at<float>(2, 2));
    tf::Transform trans_cam_tool(rot_cam_tool, tf::Vector3(tvec.at<float>(0),tvec.at<float>(1),tvec.at<float>(2)));
    tf::Transform trans_viconworld_tool =  trans_world_viconobj * trans_drone_cam * trans_cam_tool;
    tf::Vector3 target_vec(0.065,-0.065,0);
    return trans_viconworld_tool * target_vec;
}

tf::Transform compute_target_in_drone(const cv::Mat& r_mat, const cv::Mat& tvec,
const tf::Transform& trans_drone_cam) {
    tf::Matrix3x3 rot_cam_target(r_mat.at<float>(0, 0), r_mat.at<float>(0, 1), r_mat.at<float>(0, 2),
                                 r_mat.at<float>(1, 0), r_mat.at<float>(1, 1), r_mat.at<float>(1, 2),
                                 r_mat.at<float>(2, 0), r_mat.at<float>(2, 1), r_mat.at<float>(2, 2));
    tf::Transform trans_cam_target(rot_cam_target, tf::Vector3(tvec.at<float>(0),tvec.at<float>(1),tvec.at<float>(2)));

    tf::Transform trans_drone_target = trans_drone_cam * trans_cam_target;
    return trans_drone_target;
}
 
tf::Vector3 compute_delta_arm_command (const tf::Vector3& target_in_vicon,
const tf::Matrix3x3& r_base_in_vicon_world,const tf::Vector3& t_base_in_vicon_world) {
    tf::Transform trans_vicon_base(r_base_in_vicon_world, 
    tf::Vector3(t_base_in_vicon_world.x(),
                t_base_in_vicon_world.y(),
                t_base_in_vicon_world.z()));
    auto trans_base_vicon = trans_vicon_base.inverse();
    tf::Vector3 target_in_base = trans_base_vicon * target_in_vicon;
    return target_in_base;
}

class ResultPose {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d p_in_world_ob_;
        Eigen::MatrixXd p_in_world_est_;
        Eigen::MatrixXd P_;

        Eigen::Vector3d zvec_ob_;
        Eigen::MatrixXd zvec_est_;
        Eigen::MatrixXd P_zvec_;
        ResultPose() {
            p_in_world_est_ = Eigen::MatrixXd(6,1);
            p_in_world_est_.setZero();
            p_in_world_ob_.setZero();
            zvec_ob_.setZero();
            zvec_est_ = Eigen::MatrixXd(6,1);
            zvec_est_.setZero();
            P_ = Eigen::MatrixXd::Identity(6,6);
            P_zvec_ = Eigen::MatrixXd::Identity(6,6);
            for(int k = 0; k < 6; k++) {
                P_(k,k) = 0.1;
                P_zvec_(k,k) = 0.1;
            }
        };
};


#if 1
int main(int argc, char * argv[]) try
{
    CLI::App app{"aerial toolbox"};
    bool draw_frames = false;
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
    tf::Transform trans_drone_cam;
    read_camera_in_drone("/home/nuc/waypoint_ws/src/visual_sensing/cali_file/calibration_trans_drone_cam.txt", trans_drone_cam);     // for Turing

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
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ImageConverter ic(nh,rot_map,pos_map);

    double publish_frequency;
    std::string map_frame, base_frame;
    ros::Publisher position_publisher;
    ros::Publisher tool_publisher;

    ros::Publisher zvec_publisher;
    ros::Publisher debug_publisher;

    private_nh.param<double>("publish_frequency", publish_frequency, 50);
    private_nh.param<std::string>("map_frame", map_frame, "map");
    private_nh.param<std::string>("base_frame", base_frame, "camera");
    debug_publisher = nh.advertise<geometry_msgs::PoseStamped>("/pos_in_vicon_err_debug", 40);
    tool_publisher = nh.advertise<visual_sensing::toolbox_array>("/tool_box_pos", 40);

    std::string tf_prefix = "/vicon/realsense/realsense";
    ros::Rate rate(publish_frequency);
    tf::TransformListener*  tfListener= new tf::TransformListener;
    cv::Mat r_relative_mat_last = cv::Mat::eye(3,3,CV_32FC1);
    cv::Mat tvec_last = cv::Mat::zeros(3,1,CV_32FC1);
    ic.init_pts3d_toolbox();
   // exit(1);

    tf::Matrix3x3 identity(1, 0, 0,
                           0, 1, 0,
                           0, 0, 1);
    tf::Transform tf_res_final(identity, tf::Vector3(0.f,0.f,0.f));
    tf::Transform tf_res_final_upwards(identity, tf::Vector3(0.f,0.f,0.f));

    Eigen::MatrixXd s_est(6,1);
    Eigen::Vector3d p_in_viconworld;
    Eigen::MatrixXd p_in_viconworld_est(6,1);
    Eigen::Vector3d filtered_in_wold(0,0,0);
    p_in_viconworld_est.setZero();

    s_est.setZero();
    Eigen::MatrixXd P_est = Eigen::MatrixXd::Identity(6,6);
    for(int k = 0; k < 6; k++) {
        P_est(k,k) = 0.1;
    }
    Eigen::MatrixXd s_zvec_est(6,1);
    Eigen::MatrixXd P_zvec_est = Eigen::MatrixXd::Identity(6,6);
    for(int k = 0; k < 6; k++) {
        P_est(k,k) = 0.1;
    }
    const double dt = 0.0167;
    Eigen::Vector3d observation;
    Eigen::Vector3d zvec_observation;
    Eigen::Vector3d gt_world(0,0,0);
    int num_of_detection = 0;
    bool first_ob = true;
    std::vector<ResultPose> res_pose;
    res_pose.resize(8);

    tf::Vector3 p_0_Box(0.0740189,  -0.0490575,  -0.00625796);
    tf::Vector3 p_1_Box(-0.0723847,  -0.0486996,  -0.005169);// -0.0723847,  -0.0486996,  -0.005169
    tf::Vector3 p_2_Box(-0.0725707,  0.0984463,  -0.00564045);
    tf::Vector3 p_3_Box(0.074702,  0.0985078,  -0.0071995);

    std::vector<tf::Vector3> pos_tool_box;
    pos_tool_box.resize(4);
    pos_tool_box[0] = p_0_Box;
    pos_tool_box[1] = p_1_Box;
    pos_tool_box[2] = p_2_Box;
    pos_tool_box[3] = p_3_Box;
    Eigen::Vector3d p_4_target_world(-1.306, 0.021, 1.168);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        tf::StampedTransform trans_world_viconobj;
        tf::StampedTransform trans_world_turing;
        tf::StampedTransform trans_world_ruler;
        tf::StampedTransform trans_Galileo_2_world;
        tf::StampedTransform trans_Target_2_world;

        tf::StampedTransform trans_world_delta_arm;
        bool tf_ok = true;
        cv::Mat rvec = cv::Mat::zeros(3,1,CV_32FC1);
        cv::Mat r_mat = cv::Mat::eye(3,3,CV_32FC1);
        cv::Mat tvec = cv::Mat::zeros(3,1,CV_32FC1);
        ros::Time time_stamp;
        try {
            if (tfListener->canTransform("/vicon/world", "vicon/Turing/Turing", ros::Time(0))) {
                tfListener->lookupTransform("/vicon/world", "vicon/Turing/Turing", ros::Time(0), trans_world_turing);

            }
            if (tfListener->canTransform("/vicon/world", "vicon/Galileo/Galileo", ros::Time(0))) {
                tfListener->lookupTransform("/vicon/world", "vicon/Galileo/Galileo", ros::Time(0), trans_Galileo_2_world);
            }

            if (tfListener->canTransform("/vicon/world", "/vicon/ruler/ruler", ros::Time(0))) {
                tfListener->lookupTransform("/vicon/world", "/vicon/ruler/ruler", ros::Time(0), trans_world_ruler);
                gt_world(0) = trans_world_ruler.getOrigin().x();
                gt_world(1) = trans_world_ruler.getOrigin().y();
                gt_world(2) = trans_world_ruler.getOrigin().z();
            }
            tf_ok = ic.solve_pose_all(rvec,tvec,time_stamp,num_of_detection,draw_frames);
        } catch(tf::TransformException ex) {
            ROS_ERROR("-------> %s", ex.what());
            tf_ok = false;
        }


        tf::Vector3 p_B(trans_Galileo_2_world.getOrigin().x(),trans_Galileo_2_world.getOrigin().y(),trans_Galileo_2_world.getOrigin().z());
        tf::Quaternion qw1=trans_Galileo_2_world.getRotation();

        Eigen::Quaternionf q_temp;
        q_temp.w() = trans_Galileo_2_world.getRotation().w();
        q_temp.x() = trans_Galileo_2_world.getRotation().x();
        q_temp.y() = trans_Galileo_2_world.getRotation().y();
        q_temp.z() = trans_Galileo_2_world.getRotation().z();
        Eigen::Matrix3f DCM_Galileo = q_temp.toRotationMatrix();
        Eigen::Vector3f temp_ruler_vicon(trans_world_ruler.getOrigin().x(), trans_world_ruler.getOrigin().y(), trans_world_ruler.getOrigin().z());
        Eigen::Vector3f P_b_temp(trans_Galileo_2_world.getOrigin().x(),trans_Galileo_2_world.getOrigin().y(),trans_Galileo_2_world.getOrigin().z());
        Eigen::Vector3f tool_Galileo =  DCM_Galileo.transpose() * (temp_ruler_vicon - P_b_temp);


        tf::Vector3 Target_B(trans_Target_2_world.getOrigin().x(),trans_Target_2_world.getOrigin().y(),trans_Target_2_world.getOrigin().z());
        tf::Matrix3x3 rotation_matrix_body_2_world(qw1);

        if(1) {
            cv::Mat r_relative_mat;
            cv::Rodrigues(rvec,r_relative_mat);
            tf_res_final = compute_target_in_drone(r_relative_mat, tvec, trans_drone_cam);

            Eigen::Quaternionf q;
            q.w() = tf_res_final.getRotation().w();
            q.x() = tf_res_final.getRotation().x();
            q.y() = tf_res_final.getRotation().y();
            q.z() = tf_res_final.getRotation().z();
            Eigen::Matrix3f rot_mat = q.toRotationMatrix();
            visual_sensing::toolbox_array toolbox_arr;
            visual_sensing::toolbox_array toolbox_vicon;
            if (1)   // tools detection
            { 
                for (int tool_id = 0; tool_id < 4; tool_id++) {
                    Eigen::Vector3d target_tool = rot_mat.cast<double>() * toolbox_points[tool_id] + 
                                Eigen::Vector3d(tf_res_final.getOrigin().x(),
                                                tf_res_final.getOrigin().y(),
                                                tf_res_final.getOrigin().z());
                    auto temp = Eigen::Vector3d(tf_res_final.getOrigin().x(),
                                                tf_res_final.getOrigin().y(),
                                                tf_res_final.getOrigin().z());
                    observation = target_tool;
                    ic.body2world(observation, trans_world_turing, res_pose[tool_id].p_in_world_ob_); // p_in_world_ob_ before filter

                    if (first_ob) {
                        res_pose[tool_id].p_in_world_est_(0) = res_pose[tool_id].p_in_world_ob_(0);
                        res_pose[tool_id].p_in_world_est_(1) = res_pose[tool_id].p_in_world_ob_(1);
                        res_pose[tool_id].p_in_world_est_(2) = res_pose[tool_id].p_in_world_ob_(2);
                        res_pose[tool_id].P_ = Eigen::MatrixXd::Identity(6,6);
                        for(int m = 0; m < 6; m++) {
                            res_pose[tool_id].P_(m,m) = 0.1;
                        }
                        res_pose[tool_id].zvec_est_
                                << rot_mat.col(2)(0),
                                    rot_mat.col(2)(1),
                                    rot_mat.col(2)(2),
                                    0,0,0;
                        res_pose[tool_id].P_zvec_ = Eigen::MatrixXd::Identity(6,6);
                        for(int k = 0; k < 6; k++) {
                            res_pose[tool_id].P_zvec_(k,k) = 0.1;
                        }
                        first_ob = false;
                    }
                    res_pose[tool_id].zvec_ob_(0,0) = rot_mat.col(2)(0);
                    res_pose[tool_id].zvec_ob_(1,0) = rot_mat.col(2)(1);
                    res_pose[tool_id].zvec_ob_(2,0) = rot_mat.col(2)(2);

                    kf_filter(res_pose[tool_id].p_in_world_est_,
                                res_pose[tool_id].P_,
                                dt,
                                res_pose[tool_id].p_in_world_ob_);

                    toolbox_vicon.tools[tool_id].stamp = time_stamp;
                    toolbox_vicon.tools[tool_id].tool_id = tool_id;
                    tf::Vector3 pos_world_temp = p_B + rotation_matrix_body_2_world * pos_tool_box[tool_id];
                    Eigen::Vector3d pos_world_temp_E(pos_world_temp.getX(), pos_world_temp.getY(), pos_world_temp.getZ());
                    // Eigen::Vector3d pos_in_turing_body;
                    // ic.world2body(pos_world_temp_E,pos_in_turing_body);

                    toolbox_vicon.tools[tool_id].position[0] = pos_world_temp_E[0];
                    toolbox_vicon.tools[tool_id].position[1] = -pos_world_temp_E[1];
                    toolbox_vicon.tools[tool_id].position[2] = -pos_world_temp_E[2];

                    toolbox_vicon.tools[tool_id].orientation[0] = q.w();
                    toolbox_vicon.tools[tool_id].orientation[1] = q.x();
                    toolbox_vicon.tools[tool_id].orientation[2] = q.y();
                    toolbox_vicon.tools[tool_id].orientation[3] = q.z();

                    Eigen::Matrix3d rotation_body_world;
                    ic.get_rotation_matrix(rotation_body_world);
                    Eigen::Vector3d zvector = Eigen::Vector3d(0,0, -1);
                    toolbox_vicon.tools[tool_id].z_vec[0] = zvector(0);
                    toolbox_vicon.tools[tool_id].z_vec[1] = zvector(1);
                    toolbox_vicon.tools[tool_id].z_vec[2] = zvector(2);
                    toolbox_vicon.tools[tool_id].detected_num = 10;

                    // std::cout  << " toolbox_vicon.tools[tool_id].z_vec "  << std::endl;
                    // std::cout << toolbox_vicon.tools[tool_id].z_vec[0] << "   " << 
                    // toolbox_vicon.tools[tool_id].z_vec[1] << "   " <<
                    // toolbox_vicon.tools[tool_id].z_vec[2] << std::endl;

                    toolbox_arr.tools[tool_id].stamp = time_stamp;
                    toolbox_arr.tools[tool_id].tool_id = tool_id;
                    toolbox_arr.tools[tool_id].position[0] = res_pose[tool_id].p_in_world_est_(0);
                    toolbox_arr.tools[tool_id].position[1] = -res_pose[tool_id].p_in_world_est_(1);
                    toolbox_arr.tools[tool_id].position[2] = -res_pose[tool_id].p_in_world_est_(2);

                    toolbox_arr.tools[tool_id].orientation[0] = q.w();
                    toolbox_arr.tools[tool_id].orientation[1] = q.x();
                    toolbox_arr.tools[tool_id].orientation[2] = q.y();
                    toolbox_arr.tools[tool_id].orientation[3] = q.z();

                    toolbox_arr.tools[tool_id].z_vec[0] = 0.0;//res_pose[tool_id].zvec_est_(0,0);
                    toolbox_arr.tools[tool_id].z_vec[1] = 0.0;//res_pose[tool_id].zvec_est_(1,0);
                    toolbox_arr.tools[tool_id].z_vec[2] = -1.0;//res_pose[tool_id].zvec_est_(2,0);
                    toolbox_arr.tools[tool_id].detected_num = num_of_detection;

                    if(tool_id == target_id) {
                        // Eigen::Vector3d error_est;
                        // error_est(0) = res_pose[tool_id].p_in_world_est_(0) - gt_world(0);
                        // error_est(1) = res_pose[tool_id].p_in_world_est_(1) - gt_world(1);
                        // error_est(2) = res_pose[tool_id].p_in_world_est_(2) - gt_world(2);

                        // geometry_msgs::PoseStamped pos_in_vicon;
                        // pos_in_vicon.header.stamp = time_stamp;
                        // pos_in_vicon.pose.position.x = error_est(0);
                        // pos_in_vicon.pose.position.y = error_est(1);
                        // pos_in_vicon.pose.position.z = error_est(2);
                        // debug_publisher.publish(pos_in_vicon);

                        tf::Vector3 p_2_world = p_B + rotation_matrix_body_2_world * pos_tool_box[tool_id];

                        std::cout << "before KF error:" << tool_id << " " <<
                            res_pose[tool_id].p_in_world_ob_(0) - p_2_world.getX()<< " " << 
                            res_pose[tool_id].p_in_world_ob_(1) - p_2_world.getY()<< " " <<
                            res_pose[tool_id].p_in_world_ob_(2) - p_2_world.getZ()<< std::endl;
                      
                        std::cout << "\rafter KF error:"<< tool_id << " " <<
                            res_pose[tool_id].p_in_world_est_(0) - p_2_world.getX() << " " <<
                            res_pose[tool_id].p_in_world_est_(1) - p_2_world.getY() << " " <<
                            res_pose[tool_id].p_in_world_est_(2) - p_2_world.getZ() << std::endl;

                        std::cout << "num_of_detection:" << num_of_detection << std::endl;
                        if (num_of_detection > 0)
                        {
                            LOGFMTD("%s %f", "before_KF_error_x", res_pose[tool_id].p_in_world_ob_(0) - p_2_world.getX());
                            LOGFMTD("%s %f", "before_KF_error_y", res_pose[tool_id].p_in_world_ob_(1) - p_2_world.getY());
                            LOGFMTD("%s %f", "before_KF_error_z", res_pose[tool_id].p_in_world_ob_(2) - p_2_world.getZ());
                            LOGFMTD("%s %f", "after_KF_error_x", res_pose[tool_id].p_in_world_est_(0) - p_2_world.getX());
                            LOGFMTD("%s %f", "after_KF_error_y", res_pose[tool_id].p_in_world_est_(1) - p_2_world.getY());
                            LOGFMTD("%s %f", "after_KF_error_z", res_pose[tool_id].p_in_world_est_(2) - p_2_world.getZ());
                            LOGFMTD("%s %f", "after_KF_x", res_pose[tool_id].p_in_world_est_(0));
                            LOGFMTD("%s %f", "after_KF_y", res_pose[tool_id].p_in_world_est_(1));
                            LOGFMTD("%s %f", "after_KF_z", res_pose[tool_id].p_in_world_est_(2));
                            LOGFMTD("%s %f", "before_KF_x", res_pose[tool_id].p_in_world_ob_(0));
                            LOGFMTD("%s %f", "before_KF_y", res_pose[tool_id].p_in_world_ob_(1));
                            LOGFMTD("%s %f", "before_KF_z", res_pose[tool_id].p_in_world_ob_(2));
                            LOGFMTD("%s %d", "num_of_detection", num_of_detection);
                            LOGFMTD("%s %f", "vicon_x", p_2_world.getX());
                            LOGFMTD("%s %f", "vicon_y", p_2_world.getY());
                            LOGFMTD("%s %f", "vicon_z", p_2_world.getZ());
                        }
                        
                    }
                }  //end 4 tools
            }
            // #4
            toolbox_arr.tools[4].stamp = time_stamp;
            toolbox_arr.tools[4].tool_id = 4;
            toolbox_arr.tools[4].position[0] = -1.284;
            toolbox_arr.tools[4].position[1] = 1.848;
            toolbox_arr.tools[4].position[2] = -1.862;

            toolbox_arr.tools[4].orientation[0] = 1.0;
            toolbox_arr.tools[4].orientation[1] = 0.0;
            toolbox_arr.tools[4].orientation[2] = 0.0;
            toolbox_arr.tools[4].orientation[3] = 0.0;

            toolbox_arr.tools[4].z_vec[0] = 0.0;//res_pose[tool_id].zvec_est_(0,0);
            toolbox_arr.tools[4].z_vec[1] = 0.0;//res_pose[tool_id].zvec_est_(1,0);
            toolbox_arr.tools[4].z_vec[2] = -1.0;//res_pose[tool_id].zvec_est_(2,0);
            toolbox_arr.tools[4].detected_num = 10;
            // #5
            toolbox_arr.tools[5].stamp = time_stamp;
            toolbox_arr.tools[5].tool_id = 5;
            toolbox_arr.tools[5].position[0] = -1.227;
            toolbox_arr.tools[5].position[1] = 0.0;
            toolbox_arr.tools[5].position[2] = -1.783;

            toolbox_arr.tools[5].orientation[0] = 1.0;
            toolbox_arr.tools[5].orientation[1] = 0.0;
            toolbox_arr.tools[5].orientation[2] = 0.0;
            toolbox_arr.tools[5].orientation[3] = 0.0;

            toolbox_arr.tools[5].z_vec[0] = 0.0;//res_pose[tool_id].zvec_est_(0,0);
            toolbox_arr.tools[5].z_vec[1] = 0.0;//res_pose[tool_id].zvec_est_(1,0);
            toolbox_arr.tools[5].z_vec[2] = -1.0;//res_pose[tool_id].zvec_est_(2,0);
            toolbox_arr.tools[5].detected_num = 10;
            // #6
            toolbox_arr.tools[6].stamp = time_stamp;
            toolbox_arr.tools[6].tool_id = 6;
            toolbox_arr.tools[6].position[0] = -1.231;
            toolbox_arr.tools[6].position[1] = -2.049;
            toolbox_arr.tools[6].position[2] = -1.766;

            toolbox_arr.tools[6].orientation[0] = 1.0;
            toolbox_arr.tools[6].orientation[1] = 0.0;
            toolbox_arr.tools[6].orientation[2] = 0.0;
            toolbox_arr.tools[6].orientation[3] = 0.0;

            toolbox_arr.tools[6].z_vec[0] = 0.0;//res_pose[tool_id].zvec_est_(0,0);
            toolbox_arr.tools[6].z_vec[1] = 0.0;//res_pose[tool_id].zvec_est_(1,0);
            toolbox_arr.tools[6].z_vec[2] = -1.0;//res_pose[tool_id].zvec_est_(2,0);
            toolbox_arr.tools[6].detected_num = 10;
            // #7
            toolbox_arr.tools[7].stamp = time_stamp;
            toolbox_arr.tools[7].tool_id = 7;
            toolbox_arr.tools[7].position[0] = 1.0;
            toolbox_arr.tools[7].position[1] = 0.0;
            toolbox_arr.tools[7].position[2] = -1.80;

            toolbox_arr.tools[7].orientation[0] = 1.0;
            toolbox_arr.tools[7].orientation[1] = 0.0;
            toolbox_arr.tools[7].orientation[2] = 0.0;
            toolbox_arr.tools[7].orientation[3] = 0.0;

            toolbox_arr.tools[7].z_vec[0] = 0.0;//res_pose[tool_id].zvec_est_(0,0);
            toolbox_arr.tools[7].z_vec[1] = 0.0;//res_pose[tool_id].zvec_est_(1,0);
            toolbox_arr.tools[7].z_vec[2] = -1.0;//res_pose[tool_id].zvec_est_(2,0);
            toolbox_arr.tools[7].detected_num = 10;

            //use vicon
            // tool_publisher.publish(toolbox_vicon);
            // use vision
            tool_publisher.publish(toolbox_arr);
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