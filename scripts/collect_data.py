#!/usr/bin/env python
import numpy as np
import cv2
import time
import os

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class CollectDrivingData():

    def __init__(self):
        rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        rospy.Subscriber("image", Image, self.img_callback)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def cmd_callback(self, data):
        self.linear_velocity = data.linear.x
        self.angular_velocity = data.angular.z

        rospy.loginfo("LINEAR CMD : %f" , self.linear_velocity)
        rospy.loginfo("ANGULAR CMD: %f" , self.angular_velocity)

    def img_callback(self, data):
        console.log("Image Received")


def main():
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('driving_data_collector', anonymous=True)

    CollectDrivingData()
    main()