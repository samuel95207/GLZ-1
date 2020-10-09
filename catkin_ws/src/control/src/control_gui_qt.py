#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as Image_msg

import cv2
from cv_bridge import CvBridge

from PySide2.QtWidgets import *
from PySide2.QtGui import *
from PySide2.QtCore import *

import numpy as np
from PIL import Image, ImageTk

import sys
import os
import time
import threading

class ControlGUI(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)
        self.title = 'GLZ Control GUI'
        self.left = 10
        self.top = 10
        self.width = 1080
        self.height = 720

        self.video_size = QSize(640, 320)



        self.ros_setup()
        self.initUI()


        # while(True):
        #     if(self.cv_image != []):
        #         cv2.imshow('frame', self.cv_image)

        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        # cap.release()
        # cv2.destroyAllWindows()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.image_label = QLabel()
        self.image_label.setFixedSize(self.video_size)

        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.close)

        self.main_layout = QVBoxLayout()
        self.main_layout.addWidget(self.image_label)
        self.main_layout.addWidget(self.quit_button)


        self.cv_image = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.display_video_stream)
        self.timer.start(30)


        self.setLayout(self.main_layout)

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
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        # img = Image.fromarray(cv_image)



    def display_video_stream(self):
        if(self.cv_image != []):
            frame = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            frame = cv2.flip(frame, 1)
            image = QImage(frame, frame.shape[1], frame.shape[0],frame.strides[0], QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(image))


        



        






if __name__ == '__main__':  
    try:
        app = QApplication([])
        window = ControlGUI()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass