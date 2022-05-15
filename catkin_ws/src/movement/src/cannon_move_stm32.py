#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

import serial
import RPi.GPIO as GPIO


class CannonMove():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        self.firePIN = rospy.get_param('GPIO_FIRE_PIN', 4)
        GPIO.setup(self.firePIN, GPIO.OUT)

        self.ser = serial.Serial(rospy.get_param('cannon_move_py/SerialPort', "/dev/ttyACM0"), rospy.get_param('cannon_move_py/BaudRate', 9600), timeout=1)

        rospy.init_node('cannon_move_listener', anonymous=True)
        rospy.Subscriber("/"+self.glz_name+"/cannon/move", Twist, self.cannon_callback)
        rospy.Subscriber("/"+self.glz_name+"/cannon/fire", Bool, self.fire_callback)

        rospy.spin()
        

        GPIO.cleanup()

    def cannon_callback(self, data):


        yaw = data.angular.y 
        pitch = data.angular.z

        # print(yaw,end=' ')
        # print(pitch)

        sent = int(pitch*181+yaw)
        # print(sent)

        self.ser.write((str(sent)+"\n").encode())


    def fire_callback(self, data):
        # rospy.loginfo(data)                                                                                                          

        if(data.data):
            GPIO.output(self.firePIN,GPIO.HIGH)
        else:
            GPIO.output(self.firePIN,GPIO.LOW)





if __name__ == '__main__':
    bm = None
    try:
        bm = CannonMove()
    except rospy.ROSInterruptException:
        pass
