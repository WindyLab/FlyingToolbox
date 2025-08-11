#include <ctime>
#include <cstdlib>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

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

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY,30);

    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::device selected_device = selection.get_device();

    auto color_sensor = selected_device.first<rs2::color_sensor>();

    if (color_sensor.supports(RS2_OPTION_EXPOSURE)) {
        printf("color_sensor.supports(RS2_OPTION_EXPOSURE)\n");
        auto range_exposure = color_sensor.get_option_range(RS2_OPTION_EXPOSURE); 
        color_sensor.set_option(RS2_OPTION_EXPOSURE, 30);

        auto range_brightness = color_sensor.get_option_range(RS2_OPTION_BRIGHTNESS); 
        color_sensor.set_option(RS2_OPTION_BRIGHTNESS,0);

        auto range_contrast = color_sensor.get_option_range(RS2_OPTION_CONTRAST); 
        color_sensor.set_option(RS2_OPTION_CONTRAST, 100); 

        auto range_gain = color_sensor.get_option_range(RS2_OPTION_GAIN); 
        color_sensor.set_option(RS2_OPTION_GAIN, 60);


        printf("range_exposure %f %f %f %f\n",
          range_exposure.min,
          range_exposure.max,
          range_exposure.def,
          range_exposure.step);

        printf("range_gain %f %f %f %f\n",
          range_gain.min,
          range_gain.max,
          range_gain.def,
          range_gain.step);
        color_sensor.set_option(RS2_OPTION_GAIN, range_gain.max); 

          printf("constrast %f %f %f %f\n",
          range_contrast.min,
          range_contrast.max,
          range_contrast.def,
          range_contrast.step);

          printf("brightness %f %f %f %f\n",
          range_brightness.min,
          range_brightness.max,
          range_brightness.def,
          range_brightness.step);

        // printf("%f\n",range.max);
        // printf("%f\n",range.def);
        // printf("%f\n",range.step);

        // set minimal allowed exposure 
    }

    // Start streaming with default recommended configuration
   // pipe.start(cfg);


    using namespace cv;
    const auto window_name = "Display Image";
   // namedWindow(window_name, WINDOW_AUTOSIZE);

    ros::init(argc, argv, "realsense_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/realsense", 1);
    image_transport::Publisher pub_depth = it.advertise("camera/realsense_depth", 1);
    sensor_msgs::ImagePtr msg;
    sensor_msgs::ImagePtr msg_depth;

    while (nh.ok()) {
        TicToc timer;
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color = data.get_color_frame().apply_filter(color_map);
        //rs2::frame depth = data.get_depth_frame();
        auto time_stamp = ros::Time::now();

        // Query frame size (width and height)
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
       // const int w_depth = depth.as<rs2::video_frame>().get_width();
       // const int h_depth = depth.as<rs2::video_frame>().get_height();
       // std::cout << "depth height:" << h_depth << " width:" << w_depth << std::endl;

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)color.get_data());
      //  Mat depth_img(Size(w, h), CV_16UC1, (void*)depth.get_data());

        cv::cvtColor(image,image,cv::COLOR_RGB2BGR);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      //  msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();

        msg->header.stamp = time_stamp;
       // msg_depth->header.stamp = time_stamp;

        pub.publish(msg);
     //   pub_depth.publish(msg_depth);

        ros::spinOnce();
        double t = timer.toc();
        printf("current w %d h %d time for each frame%f ms\n",w,h,t);
        cv::imwrite("0_color.png",image);
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



