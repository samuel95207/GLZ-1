#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 10 16:17:55 2020

@author: 203
"""
import rospy
from std_msgs.msg import String, Float64, Header, Bool, Int32MultiArray
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image as Image_msg
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

class LEDControl():
    def __init__(self):
        self.ros_setup()

        rospy.spin()



    def ros_setup(self):
        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        rospy.init_node('led_control', anonymous=True)

        self.image_topic_name = rospy.get_param(
            "object_detection/image_topic", "/usb_cam1/image_raw/compressed")

        if("compressed" in self.image_topic_name):
            rospy.Subscriber(self.image_topic_name,
                             CompressedImage, self.image_callback)
        else:
            rospy.Subscriber(self.image_topic_name,
                             Image_msg, self.image_callback)

        self.bridge = CvBridge()

        self.led_pub = rospy.Publisher("/led", Int32MultiArray, queue_size=1)


    def image_callback(self,data):
        if("compressed" in self.image_topic_name):
            frame = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(
                data, desired_encoding='passthrough'), cv2.COLOR_BGR2RGB)
        else:
            frame = self.bridge.imgmsg_to_cv2(
                data, desired_encoding='passthrough')

                
        # image = cv2.imread('rgb_test.jpeg')
        image = frame
        width = image.shape[1]
        # print(width)
        height = image.shape[0]
        # area = width*height/10.
        region_threshold = width*height/8
        # print(height)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        # define range of green color in HSV 
        lower_green = np.array([50,100,100])
        upper_green = np.array([70,255,255])

        # define range of red color in HSV 
        lower_red = np.array([-10,100,100])
        upper_red = np.array([10,255,255])

        # Threshold the HSV image to get only blue colors
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        # Threshold the HSV image to get only green colors
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        # Threshold the HSV image to get only red colors   
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        plt.imshow(mask_red)

        color_index = [0, 0, 0, 0]
        for i in range(2):
            for j in range(2):
                blue = np.sum(mask_blue[int(i*height/2):int((i+1)*height/2), int(j*width/2):int((j+1)*width/2)])
                green = np.sum(mask_green[int(i*height/2):int((i+1)*height/2), int(j*width/2):int((j+1)*width/2)])
                red = np.sum(mask_red[int(i*height/2):int((i+1)*height/2), int(j*width/2):int((j+1)*width/2)])
                blue = blue / 255.
                green = green / 255.
                red = red / 255.
                if blue > region_threshold: color_index[2*i+j] = 1
                elif green > region_threshold: color_index[2*i+j] = 2
                elif red > region_threshold: color_index[2*i+j] = 3
                # print(blue)
                # print(green)
                # print(red)
        print(color_index)
        msg = Int32MultiArray(data=color_index)
        self.led_pub.publish(msg)

        
if __name__ == '__main__':  
    try:
        led = LEDControl()
    except rospy.ROSInterruptException:
        pass