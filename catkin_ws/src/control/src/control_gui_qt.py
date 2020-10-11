#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as Image_msg
from sensor_msgs.msg import CompressedImage

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

class ControlGUI(QWidget):

    def __init__(self,parent=None):
        super(ControlGUI, self).__init__(parent)

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
        self.title = 'GLZ Control GUI'
        self.left = 10
        self.top = 10
        self.width = 1080
        self.height = 720

        self.video_size = QSize(640, 480)


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

        self.show()


    def ros_setup(self):
        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        rospy.init_node('control_gui', anonymous=True)


        self.image_topic_name = rospy.get_param("control_gui_py/image_topic","/object_detection_image/image_raw/compressed")

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/"+self.glz_name+"/cannon/move", Twist, self.cannon_data_callback)

        if("compressed" in self.image_topic_name):
            rospy.Subscriber(self.image_topic_name, CompressedImage, self.image_callback)
        else:
            rospy.Subscriber(self.image_topic_name, Image_msg, self.image_callback)


        self.bridge = CvBridge()




    def joy_callback(self,data):
        pass
    
    def cannon_data_callback(self,data):
        pass

    def image_callback(self,data):

        if("compressed" in self.image_topic_name):
            self.cv_image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough'), cv2.COLOR_BGR2RGB)
        else:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        # img = Image.fromarray(cv_image)

       


    def display_video_stream(self):
        if(self.cv_image != []):
            frame = self.cv_image
            frame = cv2.flip(frame, 1)
            image = QImage(frame, frame.shape[1], frame.shape[0],frame.strides[0], QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(image))




if __name__ == '__main__':  
    try:
        app = QApplication(sys.argv)
        window = ControlGUI()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass