#!/usr/bin/env python3

import rospy

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import Joy
import math

class JoyControl():
    def __init__(self):

        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        self.cannon_move_pub = rospy.Publisher("/"+self.glz_name+'/cannon/move', Twist, queue_size=1)
        self.cannon_fire_pub = rospy.Publisher("/"+self.glz_name+'/cannon/fire', Bool, queue_size=1)
        self.cannon_aim_pub = rospy.Publisher("/"+self.glz_name+'/cannon/aim', Bool, queue_size=1)
        self.base_move_pub = rospy.Publisher("/"+self.glz_name+'/base/move', Twist, queue_size=1)

        rospy.init_node('joy_control', anonymous=True)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        self.yaw = 90
        self.pitch = 0

        self.yaw_rate = rospy.get_param('joy_control_py/YAW_STEP', 20)
        self.pitch_rate = rospy.get_param('joy_control_py/PITCH_STEP', 20)

        self.axes = (0,0,0,0,0,0,0,0)

        rospy.loginfo("Node joy_control has started")


        self.rate = rospy.Rate(rospy.get_param('joy_control_py/RATE', 10)) 

        while not rospy.is_shutdown():

            if(self.axes[4] > 0.9):
                self.pitch += self.pitch_rate
            elif(self.axes[4] < -0.9):
                self.pitch -= self.pitch_rate

            if(self.axes[3] > 0.9):
                self.yaw += self.yaw_rate
            elif(self.axes[3] < -0.9):
                self.yaw -= self.yaw_rate

            if(self.yaw >= 180):
                self.yaw = 180
            if(self.yaw <= 0):
                self.yaw = 0
            if(self.pitch >= 80):
                self.pitch = 80
            if(self.pitch <= 0):
                self.pitch = 0
            
            x = self.axes[1]
            if( -0.1 < x < 0.1):
                x = 0

            yaw = self.axes[0]*math.pi/2

            if( -0.2 < yaw < 0.2):
                yaw = 0

            if(self.axes[2] > -0.9):
                self.setCannon(yaw = self.yaw, pitch = self.pitch)
                
            self.setMovement(x = x, yaw=yaw)
            

            self.cannon_fire_pub.publish(self.axes[5] < -0.9)
            self.cannon_aim_pub.publish(self.axes[2] < -0.9)

            # print(self.axes)

            self.rate.sleep()
        


        

        


    def joy_callback(self,data):
        self.axes = data.axes
        self.buttons = data.buttons

        # rospy.loginfo(self.axes)


    

    def setMovement(self,x=0, yaw=0):
        cmd_vel_msg = Twist()

        cmd_vel_msg.linear.x = x
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = yaw
        cmd_vel_msg.angular.z = 0

        # rospy.loginfo(cmd_vel_msg)
        self.base_move_pub.publish(cmd_vel_msg)

    def setCannon(self,yaw=0,pitch=0):
        cmd_vel_msg = Twist()

        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = yaw
        cmd_vel_msg.angular.z = pitch

        # rospy.loginfo(cmd_vel_msg)
        self.cannon_move_pub.publish(cmd_vel_msg)


if __name__ == '__main__':  
    try:
        jc = JoyControl()
    except rospy.ROSInterruptException:
        pass