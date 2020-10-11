#!/usr/bin/env python3

import rospy

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Joy
import math

class CannonDecisionMaker():
    def __init__(self):

        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        self.cannon_move_pub = rospy.Publisher("/"+self.glz_name+'/cannon/move', Twist, queue_size=1)

        rospy.init_node('cannon_decision_maker', anonymous=True)

        rospy.Subscriber("/"+self.glz_name+'/cannon/move', Twist, self.cannon_move_callback)
        rospy.Subscriber("/object_detection/lock_pos", Point, self.lock_pos_callback)

        self.image_width = rospy.get_param("object_detection/image_width", 640)
        self.image_height = rospy.get_param("object_detection/image_height", 480)

        
        self.yaw = None
        self.pitch = None

        self.yaw_rate = rospy.get_param('cannon_decision_maker_py/YAW_STEP', 2)
        self.pitch_rate = rospy.get_param('cannon_decision_maker_py/PITCH_STEP', 2)


        self.rate = rospy.Rate(rospy.get_param('cannon_decision_maker_py/RATE', 10)) 

        while not rospy.is_shutdown():

            if(self.yaw == None or self.pitch == None):
                continue

            if(self.yaw >= 180):
                self.yaw = 180
            if(self.yaw <= 0):
                self.yaw = 0
            if(self.pitch >= 80):
                self.pitch = 80
            if(self.pitch <= 0):
                self.pitch = 0
            
            self.setCannon(yaw = self.yaw, pitch = self.pitch)

            # print(self.axes)

            self.rate.sleep()
        


        
    def lock_pos_callback(self,data):
        rospy.loginfo(data)
        print(self.image_width/2*1.1)
        print(self.image_width/2*0.9)
        if(data.x > self.image_width/2*1.1):
            self.yaw += self.yaw_rate
        elif(data.x < self.image_width/2*0.9):
            self.yaw -= self.yaw_rate
        


    def cannon_move_callback(self,data):
        self.yaw = data.angular.y
        self.pitch = data.angular.z

        # rospy.loginfo(self.axes)



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
        cdm = CannonDecisionMaker()
    except rospy.ROSInterruptException:
        pass