#!/usr/bin/env python3
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import cmd
from geometry_msgs.msg import TransformStamped
import message_filters
import tf
import pdb

R_proj = np.array([[607.019, 0, 315.875],[0, 607.019, 248.763],[0, 0, 1]])
distCoeffs = np.float32([0.182874, -0.56215, 0.000804634, -0.00028305])
R_camera2hand = tf.transformations.quaternion_matrix(np.array([-0.668156, 0.741952, -0.00560952, 0.0551651]))[0:3, 0:3]
T_camera2hand = np.array([0.121479, -0.0270956, -0.182878]).reshape([3, 1])


M_camera2hand = np.hstack([R_camera2hand, T_camera2hand])
M_camera2hand = np.vstack([M_camera2hand, np.array([0, 0, 0, 1])])

M_hand2camera = np.linalg.inv(M_camera2hand)
R_hand2camera = M_hand2camera[0:3, 0:3]
T_hand2camera = M_hand2camera[0:3, 3].reshape([3, 1])
print(T_hand2camera)

p_0_Box = np.array([0.0651373,  -0.0551356,  -0.00835383]).reshape([3, 1])
p_1_Box = np.array([-0.0811293,  -0.0456729,  -0.00489738]).reshape([3, 1])
p_2_Box = np.array([-0.0743545,  0.0988536,  -0.0101433]).reshape([3, 1])
p_3_Box = np.array([0.0740469,  0.091118,  -0.00785489]).reshape([3, 1])


def callback(camera_data, hand_data, target_data, tool_data):
    global count, bridge
    if count <= 100000:
            print(count)
            cv_img = bridge.imgmsg_to_cv2(camera_data, "bgr8")
            timestr = "%.6f" %  camera_data.header.stamp.to_sec()
            image_name = str(count) + ".jpg"

            img_h = 640
            img_w = 480
            #Coordinate Transformation
            right_img = cv_img
            #right_img = cv2.undistort(right_img, R_proj, distCoeffs)
            ori_image = right_img
            #Hand R T
            quat_hand = np.array([hand_data.transform.rotation.x, hand_data.transform.rotation.y, hand_data.transform.rotation.z, hand_data.transform.rotation.w])
            trans_hand = np.array([hand_data.transform.translation.x, hand_data.transform.translation.y, hand_data.transform.translation.z])
            R_hand2vicon = tf.transformations.quaternion_matrix(quat_hand)[0:3, 0:3]
            T_hand2vicon = trans_hand.reshape([3, 1])

            #print(quaternion_hand)
            #print(R_hand2vicon, T_hand2vicon)
            M_hand2vicon = np.hstack([R_hand2vicon, T_hand2vicon.reshape([3, 1])])
            M_hand2vicon = np.vstack([M_hand2vicon, np.array([0, 0, 0, 1])])

            M_vicon2hand = np.linalg.inv(M_hand2vicon)

            #Target R T
            quat_target = np.array([target_data.transform.rotation.x, target_data.transform.rotation.y, target_data.transform.rotation.z, target_data.transform.rotation.w])
            trans_target = np.array([target_data.transform.translation.x, target_data.transform.translation.y, target_data.transform.translation.z])
            R_target2vicon = tf.transformations.quaternion_matrix(quat_target)[0:3, 0:3]
            T_target2vicon = trans_target.reshape([3, 1])

            M_target2vicon = np.hstack([R_target2vicon, T_target2vicon.reshape([3, 1])])
            M_target2vicon = np.vstack([M_target2vicon, np.array([0, 0, 0, 1])])

            M_vicon2target = np.linalg.inv(M_target2vicon)

            #Tool R T
            quat_tool = np.array([tool_data.transform.rotation.x, tool_data.transform.rotation.y, tool_data.transform.rotation.z, tool_data.transform.rotation.w])
            trans_tool = np.array([tool_data.transform.translation.x, tool_data.transform.translation.y, tool_data.transform.translation.z])
            R_tool2vicon = tf.transformations.quaternion_matrix(quat_tool)[0:3, 0:3]
            T_tool2vicon = trans_tool.reshape([3, 1])

            M_tool2vicon = np.hstack([R_tool2vicon, T_tool2vicon.reshape([3, 1])])
            M_tool2vicon = np.vstack([M_tool2vicon, np.array([0, 0, 0, 1])])

            # Marker Position
            marker_pos = T_target2vicon
            print(marker_pos)
            p_o2h = np.linalg.inv(R_hand2vicon).dot(marker_pos - T_hand2vicon)
            #print(p_o2h)
            p_o2c = R_hand2camera.dot(p_o2h) + T_hand2camera
            #print(p_o2c)
            #print(M_hand2camera[0:3, 0:3], M_hand2camera[0:3, 3])
            m_proj = R_proj.dot(p_o2c)
            #print(m_proj)
            m_proj = m_proj / m_proj[2]

            print(m_proj)

            #Tool_0_box position
            tool_0_pos = R_tool2vicon.dot(p_0_Box) + T_tool2vicon
            tool_1_pos = R_tool2vicon.dot(p_1_Box) + T_tool2vicon
            tool_2_pos = R_tool2vicon.dot(p_2_Box) + T_tool2vicon
            tool_3_pos = R_tool2vicon.dot(p_3_Box) + T_tool2vicon

            tool_0_o2h = np.linalg.inv(R_hand2vicon).dot(tool_0_pos - T_hand2vicon)
            tool_1_o2h = np.linalg.inv(R_hand2vicon).dot(tool_1_pos - T_hand2vicon)
            tool_2_o2h = np.linalg.inv(R_hand2vicon).dot(tool_2_pos - T_hand2vicon)
            tool_3_o2h = np.linalg.inv(R_hand2vicon).dot(tool_3_pos - T_hand2vicon)

            tool_0_o2c = R_hand2camera.dot(tool_0_o2h) + T_hand2camera
            tool_1_o2c = R_hand2camera.dot(tool_1_o2h) + T_hand2camera
            tool_2_o2c = R_hand2camera.dot(tool_2_o2h) + T_hand2camera
            tool_3_o2c = R_hand2camera.dot(tool_3_o2h) + T_hand2camera

            tool_0_proj = R_proj.dot(tool_0_o2c)
            tool_1_proj = R_proj.dot(tool_1_o2c)
            tool_2_proj = R_proj.dot(tool_2_o2c)
            tool_3_proj = R_proj.dot(tool_3_o2c)

            tool_0_proj = tool_0_proj / tool_0_proj[2]
            tool_1_proj = tool_1_proj / tool_1_proj[2]
            tool_2_proj = tool_2_proj / tool_2_proj[2]
            tool_3_proj = tool_3_proj / tool_3_proj[2]



            right_img = cv2.circle(right_img, (int(m_proj[0]), int(m_proj[1])), 4, (255, 0, 0), 2)

            right_img = cv2.circle(right_img, (int(tool_0_proj[0]), int(tool_0_proj[1])), 1, (255, 255, 0), 2)
            right_img = cv2.circle(right_img, (int(tool_1_proj[0]), int(tool_1_proj[1])), 1, (255, 255, 0), 2)
            right_img = cv2.circle(right_img, (int(tool_2_proj[0]), int(tool_2_proj[1])), 1, (255, 255, 0), 2)
            right_img = cv2.circle(right_img, (int(tool_3_proj[0]), int(tool_3_proj[1])), 1, (255, 255, 0), 2)
            count += 1
            cv2.imwrite('test.jpg', right_img)
    else:
        pass
 

def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)
 
    # make a video_object and init the video object
    global count, bridge
    count = 0
    bridge = CvBridge()

    # camera_img = message_filters.Subscriber('/usb_cam/image_raw', Image, queue_size=1)
    camera_img = message_filters.Subscriber('/camera/realsense', Image, queue_size=1)
    hand = message_filters.Subscriber('/vicon/Turing/Turing', TransformStamped, queue_size=1)
    target = message_filters.Subscriber('/vicon/ruler/ruler', TransformStamped, queue_size=1)

    tool_box = message_filters.Subscriber('/vicon/Galileo/Galileo', TransformStamped, queue_size=1)
    
    ts = message_filters.ApproximateTimeSynchronizer([camera_img, hand, target, tool_box], 10, 0.1, allow_headerless = True)
    ts.registerCallback(callback)

    rospy.spin()
 
if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        displayWebcam()
    
