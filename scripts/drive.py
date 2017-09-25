#!/usr/bin/env python
#resources sensor_msgs/CompressedImage to OpenCV Image http://docs.erlerobotics.com/robot_operating_system/ros/basic_concepts/examples/subcribe_images
#training https://github.com/hamuchiwa/AutoRCCar/blob/master/computer/collect_training_data.py
import cv2
import numpy as np
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

class NeuralNetwork(object):
    def __init__(self):
        self.model = cv2.ANN_MLP()

    def create(self):
        layer_size = np.int32([38400, 32, 4])
        self.model.create(layer_size)
        self.model.load('mlp.xml')

    def predict(self, samples):
        ret, resp = self.model.predict(samples)
        return resp.argmax(-1)

class Drive():

    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, self.img_callback)

        self.image_array = np.zeros((1, 38400))
        self.label_array = np.zeros((1, 4), 'float')

        self.model = NeuralNetwork()
        self.model.create()        

    def img_callback(self, data):
        # rospy.loginfo("Image Received")
        np_arr = np.fromstring(data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)

        roi = image[120:240, :]
        temp_image_array = roi.reshape(1, 38400).astype(np.float32)

        prediction = self.model.predict(temp_image_array)
        self.drive(prediction)

    def drive(self, prediction):
        cmd = Twist()

        #forward
        if prediction == 2:
            cmd.linear.x = 0.5

        #steer left
        elif prediction == 0:
            cmd.linear.x = 0.5
            cmd.angular.z = 1.0

        #steer right
        elif prediction == 1:
            cmd.linear.x = 0.5
            cmd.angular.z = -1.0

        #stop
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main():
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_follower', anonymous=True)
    print "Driving..."
    Drive()
    main()
