#include "../include/qnode.hpp"

namespace airbox_monitor {

namespace {
    // Static variables for storing current poses
    geometry_msgs::TransformStamped current_pose_uav1_;
    geometry_msgs::TransformStamped current_pose_uav2_;
}

QNode::QNode(int argc, char** argv)
    : init_argc_(argc)
    , init_argv_(argv)
{
}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

void QNode::UAV1_pose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    current_pose_uav1_ = *msg;
    Q_EMIT UAV1_updatePose(current_pose_uav1_);
}

void QNode::UAV2_pose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    current_pose_uav2_ = *msg;
    Q_EMIT UAV2_updatePose(current_pose_uav2_);
}

bool QNode::init() {
    // Initialize ROS node
    ros::init(init_argc_, init_argv_, "airbox_monitor_node");
    if (!ros::master::check()) {
        return false;
    }
    ros::start();

    // Setup node handles
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");  // Private node handle for parameters

    // Get topic names from parameters
    std::string topic1, topic2;
    private_nh.param<std::string>("topic1", topic1, "/vicon/Turing/Turing");
    private_nh.param<std::string>("topic2", topic2, "/vicon/Galileo/Galileo");
    private_nh.param<double>("collision_threshold", collision_threshold_, 1.0);
    ROS_INFO("Collision detection threshold set to: %.2f meters", collision_threshold_);
    ROS_INFO("topic1 is: %s", topic1.c_str());
    ROS_INFO("topic2 is: %s", topic2.c_str());

    // Subscribe to UAV pose topics
    uav1_pose_sub_ = n.subscribe<geometry_msgs::TransformStamped>(
        topic1, 10, &QNode::UAV1_pose_cb, this);
    uav2_pose_sub_ = n.subscribe<geometry_msgs::TransformStamped>(
        topic2, 10, &QNode::UAV2_pose_cb, this);

    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("ROS shutdown, closing GUI.");
    Q_EMIT rosShutdown();
}

}  // namespace airbox_monitor