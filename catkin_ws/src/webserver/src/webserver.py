#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as Image_msg
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image, ImageTk
from threading import Thread, Event
import time

from flask import Flask, render_template, Response, request


class ROSControl:
    def __init__(self):
        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        
        Thread(target=lambda: self.ros_run()).start()

        self.cv_image = np.zeros(shape=[512, 512, 3], dtype=np.uint8)

        self.image_topic_name = rospy.get_param(
            "webserver/image_topic", "/usb_cam1/image_raw/compressed")

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/"+self.glz_name+"/cannon/move",
                         Twist, self.cannon_data_callback)

        if("compressed" in self.image_topic_name):
            rospy.Subscriber(self.image_topic_name,
                             CompressedImage, self.image_callback)
        else:
            rospy.Subscriber(self.image_topic_name,
                             Image_msg, self.image_callback)

        rospy.Subscriber("/"+self.glz_name+"/cannon/move", Twist, self.cannon_callback)
        rospy.Subscriber("/"+self.glz_name+"/base/move", Twist, self.base_callback)


        self.cannon_yaw_read = 0
        self.cannon_pitch_read = 0
        self.base_x_read = 0
        self.base_yaw_read = 0


        self.bridge = CvBridge()

        time.sleep(3)


    

    
        
    def ros_run(self):
        rospy.init_node('webserver', disable_signals=True)
        self.cannon_move_pub = rospy.Publisher("/"+self.glz_name+'/cannon/move', Twist, queue_size=1)
        self.cannon_fire_pub = rospy.Publisher("/"+self.glz_name+'/cannon/fire', Bool, queue_size=1)
        self.cannon_aim_pub = rospy.Publisher("/"+self.glz_name+'/cannon/aim', Bool, queue_size=1)
        self.base_move_pub = rospy.Publisher("/"+self.glz_name+'/base/move', Twist, queue_size=1)
        rospy.spin()

    def cannon_callback(self,data):
        self.cannon_yaw_read = data.angular.y
        self.cannon_pitch_read = data.angular.z


    def base_callback(self,data):
        self.base_x_read = data.linear.x
        self.base_yaw_read = data.angular.y

    def joy_callback(self, data):
        pass

    def cannon_data_callback(self, data):
        pass

    def image_callback(self, data):

        if("compressed" in self.image_topic_name):
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
                data, desired_encoding='passthrough')
        else:
            self.cv_image = self.bridge.imgmsg_to_cv2(
                data, desired_encoding='passthrough')
        # img = Image.fromarray(cv_image)

    def get_frame(self):
        ret, jpeg = cv2.imencode('.jpg', self.cv_image)
        frame = jpeg.tobytes()
        return frame

    def setCannon(self,yaw=None,pitch=None):
        cmd_vel_msg = Twist()

        if(yaw == None):
            yaw = self.cannon_yaw_read
        if(pitch == None):
            pitch = self.cannon_pitch_read

        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = yaw
        cmd_vel_msg.angular.z = pitch

        # rospy.loginfo(cmd_vel_msg)
        self.cannon_move_pub.publish(cmd_vel_msg)

    def setMovement(self,x=None, yaw=None):
        cmd_vel_msg = Twist()

        if(x == None):
            x = self.base_x_read
        if(yaw == None):
            yaw = self.base_yaw_read

        cmd_vel_msg.linear.x = x
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = yaw
        cmd_vel_msg.angular.z = 0

        # rospy.loginfo(cmd_vel_msg)
        self.base_move_pub.publish(cmd_vel_msg)


# flask
app = Flask(__name__)

rosc = ROSControl()


@app.route('/')
def index():
    return render_template('index.html')


def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(rosc),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/cannon/yaw/<int:yaw>')
def cannon_yaw_api(yaw):
    # rosc.setCannon(yaw=yaw, pitch=None)
    rosc.setMovement(yaw=yaw, x=None)


@app.route('/api/cannon/pitch/<int:pitch>')
def cannon_pitch_api(pitch):
    rosc.setCannon(yaw=None, pitch=pitch)
    time.sleep(1)
    rosc.setCannon(yaw=None, pitch=0)


@app.route('/api/cannon/fire/<int:fire>')
def cannon_fire_api(fire):
    if(not(fire==0)):
        rosc.cannon_fire_pub.publish(True)
        print("fire")
        time.sleep(0.7)
        rosc.cannon_fire_pub.publish(False)


@app.route('/api/base/x/<float:x>')
def base_x_api(x):
    t = x / 0.05
    rosc.setMovement(yaw=None, x=1)
    time.sleep(t)
    rosc.setMovement(yaw=None, x=0)

@app.route('/api/base/xr/<float:x>')
def base_xr_api(x):
    t = x / 0.5
    rosc.setMovement(yaw=None, x=-1)
    time.sleep(t)
    rosc.setMovement(yaw=None, x=0)


@app.route('/api/base/yaw/<int:yaw>')
def base_yaw_api(yaw):
    yaw-=90
    yaw=yaw/180*3.14 
    rosc.setMovement(yaw=yaw, x=None)
    time.sleep(1)
    rosc.setMovement(yaw=0, x=None)







if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, use_reloader=False)
