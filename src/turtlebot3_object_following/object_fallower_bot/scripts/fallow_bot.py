#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from audioop import error

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class robot_camera:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("fallow_bot_node")
        rospy.loginfo("fallow_bot_node started")

        # Subscribe to the camera topic
        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_person)

        # Publisher for robot velocity commands
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Initialize the speed and CV bridge
        self.speed = Twist()
        self.bridge = CvBridge()

        rospy.spin()

    def camera_person(self, msg):
        self.cap = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cap = cv2.resize(self.cap, (640, 480))
        cv2.imshow("frame", self.cap)

        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        (regions, _) = hog.detectMultiScale(
            self.cap,
            winStride=(4, 4),
            padding=(4, 4),
            scale=1.05
        )
        count = len(regions)

        if count > 0:
            x, y, w, h = regions[0]

            center_x = x + w / 2
            center_y = y + h / 2

            error_x = center_x - self.cap.shape[1] / 2

            cv2.rectangle(self.cap, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(self.cap, (int(center_x), int(center_y)), 5, (0, 0, 255), 2)

            self.speed.linear.x = 0.4

            self.speed.angular.z = -error_x / 250  # Reduced sensitivity
            self.speed.angular.z = max(min(self.speed.angular.z, 0.2), -0.2)  # Limit angular velocity range

            self.pub.publish(self.speed)
            rospy.loginfo(f"Going to person: {self.speed}")
        else:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.2

            self.pub.publish(self.speed)
            rospy.loginfo("Searching for person")

        cv2.imshow("frame", self.cap)
        cv2.waitKey(1)


if __name__ == "__main__":
    obj = robot_camera()
