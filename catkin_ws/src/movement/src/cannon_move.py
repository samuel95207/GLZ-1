#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from Servo import *


class CannonMove():
    def __init__(self):

        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')
        self.pin_config = {"GPIO_YAW_PIN": rospy.get_param('cannon_move_py/GPIO_YAW_PIN', 16),
                           "GPIO_PITCH_PIN": rospy.get_param('cannon_move_py/GPIO_PITCH_PIN', 17)
                           }


        self.yaw_servo = Servo(self.pin_config["GPIO_YAW_PIN"])
        self.pitch_servo = Servo(self.pin_config["GPIO_PITCH_PIN"])

        rospy.init_node('cannon_move_listener', anonymous=True)
        rospy.Subscriber("/"+self.glz_name+"/cannon/move", Twist, self.cannon_callback)

        rospy.spin()

    def cannon_callback(self, data):
        # rospy.loginfo(data)
        if(0 <= data.angular.y <= 180):
            self.yaw_servo.write(data.angular.y)
        if(0 <= data.angular.z <= 80):
            self.pitch_servo.write(data.angular.z)


if __name__ == '__main__':
    try:
        bm = CannonMove()
    except rospy.ROSInterruptException:
        pass
