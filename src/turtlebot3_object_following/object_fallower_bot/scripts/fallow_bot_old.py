#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from audioop import error

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class robot_camera():
    def __init__(self):

        rospy.init_node("fallow_bot_node")
        rospy.loginfo("fallow_bot_node started")

        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_person)

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.speed = Twist()
        self.bridge = CvBridge()

        rospy.spin()

    def camera_person(self, msg):
        self.cap = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.cap = cv2.resize(self.cap, (640, 480))
        cv2.imshow("frame", self.cap)

        # Hog descriptor
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        (regions, _) = (hog.detectMultiScale(self.cap,
                                             winStride=(4, 4),
                                             padding=(4, 4),
                                             scale=1.05))
        count = len(regions)
        for (x, y, w, h) in regions:
            cv2.rectangle(self.cap, (x, y),
                          (x + w, y + h),
                          (0, 255, 0), 1)

        # Fill your code in here 
        

        cv2.imshow("frame", self.cap)
        cv2.waitKey(1)


if __name__ == "__main__":
    obj = robot_camera()
