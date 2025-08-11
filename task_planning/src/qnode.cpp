#include "../include/qnode.hpp"

namespace task_planning {

namespace {
    // Static variables for storing current poses
    geometry_msgs::TransformStamped current_pose_MAV1_;
    geometry_msgs::TransformStamped current_pose_MAV2_;
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

void QNode::MAV1_pose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    current_pose_MAV1_ = *msg;
    Q_EMIT MAV1_updatePose(current_pose_MAV1_);
}

void QNode::MAV2_pose_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    current_pose_MAV2_ = *msg;
    Q_EMIT MAV2_updatePose(current_pose_MAV2_);
}

bool QNode::init() {
    // Initialize ROS node
    ros::init(init_argc_, init_argv_, "task_planning_node");
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

    // Subscribe to MAV pose topics
    MAV1_pose_sub_ = n.subscribe<geometry_msgs::TransformStamped>(
        topic1, 10, &QNode::MAV1_pose_cb, this);
    MAV2_pose_sub_ = n.subscribe<geometry_msgs::TransformStamped>(
        topic2, 10, &QNode::MAV2_pose_cb, this);

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

}  // namespace task_planning