#!/usr/bin/env python3

import rospy

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as Image_msg

import cv2
from cv_bridge import CvBridge

import tkinter as tk
import numpy as np
from PIL import Image, ImageTk
import time


class ControlGUI:
    def __init__(self):
        self.tkmaster = tk.Tk()
        self.tkmaster.title("GLZ Control")


        self.components_render()
        self.ros_setup()


        self.video_stream()



        self.tkmaster.mainloop()

    def components_render(self):
        self.imageDisplay = tk.Label(self.tkmaster, bg="White", height=480, width=600)
        self.imageDisplay.pack() 

        self.imgtk = []

    def ros_setup(self):
        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        rospy.init_node('control_gui', anonymous=True)

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/"+self.glz_name+"/cannon/move", Twist, self.cannon_data_callback)
        rospy.Subscriber(rospy.get_param("control_gui_py/image_topic","/usb_cam/image_raw"), Image_msg, self.image_callback)

        self.bridge = CvBridge()

    def joy_callback(self):
        pass
    
    def cannon_data_callback(self):
        pass

    def image_callback(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        cv2image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(cv2image)
        self.imgtk = ImageTk.PhotoImage(image=img)

    def video_stream(self):
        if(self.imgtk != []):
            self.imageDisplay.imgtk = self.imgtk
            self.imageDisplay.configure(image=self.imgtk)
            print("in")
        self.imageDisplay.after(1,self.video_stream)

        






if __name__ == '__main__':  
    try:
        cg = ControlGUI()
    except rospy.ROSInterruptException:
        pass