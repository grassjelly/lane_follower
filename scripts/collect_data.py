#!/usr/bin/env python
#resources sensor_msgs/CompressedImage to OpenCV Image http://docs.erlerobotics.com/robot_operating_system/ros/basic_concepts/examples/subcribe_images
#training https://github.com/hamuchiwa/AutoRCCar/blob/master/computer/collect_training_data.py
import numpy as np
import cv2
import time
import os

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

class CollectDrivingData():

    def __init__(self):
        # rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        # rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, self.img_callback)

        rospy.Subscriber("cmd_vel_mux/input/teleop", Twist, self.cmd_callback)
        rospy.Subscriber("camera/rgb/image_raw/compressed", CompressedImage, self.img_callback)
        rospy.Subscriber("save_data", Bool, self.save_callback)

        self.temp_image_array = 0.0
        self.image_array = np.zeros((1, 38400))
        self.label_array = np.zeros((1, 4), 'float')

        self.frame = 0
        self.total_frame = 0
        self.saved_frame = 0

        self.k = np.zeros((4,4), 'float')
        for i in range(4):
            self.k[i, i] = 1            

    def save_callback(self,data):
        #rostopic pub save_data std_msgs/Bool true -1
        if(data.data):
            print "Saving data"
            self.save_collected_data()


    def cmd_callback(self, data):
        linear_velocity = data.linear.x
        angular_velocity = data.angular.z
        trigger = data.linear.y
        # rospy.loginfo("LINEAR CMD : %f" , linear_velocity)
        # rospy.loginfo("ANGULAR CMD: %f" , angular_velocity)
        # rospy.loginfo("SAVED FRAME: %f" , self.saved_frame)
        # rospy.loginfo("TOTAL FRAME: %f" , self.total_frame)

        #steer right
        if linear_velocity > 0.0 and angular_velocity < 0.0:
            self.image_array = np.vstack((self.image_array, self.temp_image_array))
            self.label_array = np.vstack((self.label_array, self.k[1]))
            self.saved_frame +=1

        #steer left
        elif linear_velocity > 0.0 and angular_velocity > 0.0:
            self.image_array = np.vstack((self.image_array, self.temp_image_array))
            self.label_array = np.vstack((self.label_array, self.k[0]))
            self.saved_frame +=1

        #forward
        elif linear_velocity > 0.0:
            self.image_array = np.vstack((self.image_array, self.temp_image_array))
            self.label_array = np.vstack((self.label_array, self.k[2]))
            self.saved_frame +=1

        #reverse
        elif linear_velocity < 0.0:
            self.image_array = np.vstack((self.image_array, self.temp_image_array))
            self.label_array = np.vstack((self.label_array, self.k[3]))
            self.saved_frame +=1
        

    def img_callback(self, data):
        # rospy.loginfo("Image Received")
        np_arr = np.fromstring(data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)

        roi = image[120:240, :]
        cv2.imshow('image', image)

        self.temp_image_array = roi.reshape(1, 38400).astype(np.float32)

        self.frame += 1                    
        self.total_frame += 1

    def save_collected_data(self):

        train = self.image_array[1:, :]
        train_labels = self.label_array[1:, :]
        
        print train.shape
        print train_labels.shape

        filename = str(int(time.time()))
        #TODO: parametize this
        try:
            np.savez(filename + '.npz', train=train, train_labels=train_labels)
        except IOError as e:
            print e
        
        exit(0)

def main():
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('driving_data_collector', anonymous=True)
    print "Start Driving to collect data"
    CollectDrivingData()
    main()