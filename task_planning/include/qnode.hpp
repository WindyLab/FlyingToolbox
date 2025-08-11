/**
 * @file /include/airbox_monitor/qnode.hpp
 * @brief ROS communication node for UAV monitoring
 */

#ifndef AIRBOX_MONITOR_QNODE_HPP_
#define AIRBOX_MONITOR_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string>
#include <QThread>
#include <QStringListModel>
#include <QDebug>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace airbox_monitor {

class QNode : public QThread {
    Q_OBJECT
public:
    // Constructor & Destructor
    QNode(int argc, char** argv);
    virtual ~QNode();

    // Public methods
    bool init();
    void run();

    // ROS Callbacks
    void UAV1_pose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void UAV2_pose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    double getCollisionThreshold() const { return collision_threshold_; }

Q_SIGNALS:
    void UAV1_updatePose(const geometry_msgs::TransformStamped& pose);
    void UAV2_updatePose(const geometry_msgs::TransformStamped& pose);
    void rosShutdown();

private:
    int init_argc_;
    char** init_argv_;
    double collision_threshold_;  // Store collision threshold from parameter server
    ros::Subscriber uav1_pose_sub_;
    ros::Subscriber uav2_pose_sub_;
};

}  // namespace airbox_monitor

#endif  // AIRBOX_MONITOR_QNODE_HPP_