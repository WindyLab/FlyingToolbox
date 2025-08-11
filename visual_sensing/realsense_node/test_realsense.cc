#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// Simple timer class for measuring frame processing time
class Timer {
public:
    Timer() { start(); }
    void start() { t0 = std::chrono::system_clock::now(); }
    double elapsed() {
        auto t1 = std::chrono::system_clock::now();
        std::chrono::duration<double> d = t1 - t0;
        return d.count() * 1000;
    }
private:
    std::chrono::time_point<std::chrono::system_clock> t0;
};

int main(int argc, char* argv[]) try {
    // Create RealSense colorizer for color frames
    rs2::colorizer colorizer;
    // Create RealSense pipeline
    rs2::pipeline pipeline;
    // Configure color stream parameters
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);

    // Start pipeline and get device
    auto profile = pipeline.start(config);
    auto device = profile.get_device();
    auto sensor = device.first<rs2::color_sensor>();

    // Set camera parameters (exposure, brightness, contrast, gain)
    if (sensor.supports(RS2_OPTION_EXPOSURE)) {
        auto exp_range = sensor.get_option_range(RS2_OPTION_EXPOSURE);
        sensor.set_option(RS2_OPTION_EXPOSURE, 30);

        auto bright_range = sensor.get_option_range(RS2_OPTION_BRIGHTNESS);
        sensor.set_option(RS2_OPTION_BRIGHTNESS, 0);

        auto contrast_range = sensor.get_option_range(RS2_OPTION_CONTRAST);
        sensor.set_option(RS2_OPTION_CONTRAST, 100);

        auto gain_range = sensor.get_option_range(RS2_OPTION_GAIN);
        sensor.set_option(RS2_OPTION_GAIN, gain_range.max);

        // Print parameter ranges
        printf("Exposure: %f %f %f %f\n", exp_range.min, exp_range.max, exp_range.def, exp_range.step);
        printf("Gain: %f %f %f %f\n", gain_range.min, gain_range.max, gain_range.def, gain_range.step);
        printf("Contrast: %f %f %f %f\n", contrast_range.min, contrast_range.max, contrast_range.def, contrast_range.step);
        printf("Brightness: %f %f %f %f\n", bright_range.min, bright_range.max, bright_range.def, bright_range.step);
    }

    // Initialize ROS node and create image publisher
    ros::init(argc, argv, "rs_pub");
    ros::NodeHandle node;
    image_transport::ImageTransport img_trans(node);
    auto color_pub = img_trans.advertise("camera/rs_color", 1);

    // Main loop: capture, process, and publish color images
    while (node.ok()) {
        Timer t; // Start timer
        auto frames = pipeline.wait_for_frames();
        auto color_frame = frames.get_color_frame().apply_filter(colorizer);
        auto stamp = ros::Time::now();

        int width = color_frame.as<rs2::video_frame>().get_width();
        int height = color_frame.as<rs2::video_frame>().get_height();

        // Convert RealSense frame to OpenCV image
        cv::Mat color_img(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data());
        cv::cvtColor(color_img, color_img, cv::COLOR_RGB2BGR);

        // Convert to ROS image message and publish
        auto ros_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_img).toImageMsg();
        ros_img->header.stamp = stamp;

        color_pub.publish(ros_img);

        ros::spinOnce();
        double ms = t.elapsed(); // Calculate processing time
        printf("Frame: %dx%d, Time: %.2f ms\n", width, height, ms);

        // Save current frame to local file
        cv::imwrite("0_color.png", color_img);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e) {
    std::cerr << "RealSense error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}