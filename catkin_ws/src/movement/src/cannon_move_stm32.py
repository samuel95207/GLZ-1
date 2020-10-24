#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

import serial
import rpi.GPIO as GPIO


class CannonMove():
    def __init__(self):

        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        self.ser = serial.Serial(rospy.get_param('cannon_move_py/SerialPort', "/dev/ttyS1"), rospy.get_param('cannon_move_py/BaudRate', 9600), timeout=1)

        rospy.init_node('cannon_move_listener', anonymous=True)
        rospy.Subscriber("/"+self.glz_name+"/cannon/move", Twist, self.cannon_callback)
        rospy.Subscriber("/"+self.glz_name+"/cannon/fire", Bool, self.fire_callback)

        rospy.spin()
        

        GPIO.cleanup()

    def cannon_callback(self, data):
        # rospy.loginfo(data)
        if(0 <= data.angular.y <= 180):
            self.ser.write((f'RawServo={data.angular.y}').encode())
        if(0 <= data.angular.z <= 80):
            self.ser.write((f'PitchServo={data.angular.z}').encode())


    def fire_callback(self, data):
        # rospy.loginfo(data)                                                                                                          

        if(data.data):
            GPIO.output(self.pin_config["GPIO_FIRE_PIN"],GPIO.HIGH)
        else:
            GPIO.output(self.pin_config["GPIO_FIRE_PIN"],GPIO.LOW)





if __name__ == '__main__':
    bm = None
    try:
        bm = CannonMove()
    except rospy.ROSInterruptException:
        pass
