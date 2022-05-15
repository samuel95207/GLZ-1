#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist

import math
import RPi.GPIO as GPIO
import time


class LED():
    def __init__(self):
        self._glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')
        self._pin_config = {"EA_PIN": rospy.get_param('base_move_py/EA_PIN',3),
                            "PWM_PIN": rospy.get_param('base_move_py/PWM_PIN',4),
                            }

        self.delay = rospy.get_param('base_move_py/DELAY',50)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin_config["EA_PIN"], GPIO.OUT)



        self.PWM_FREQ = 1000
        self.pwm = GPIO.PWM(self._pin_config["PWM_PIN"], self.PWM_FREQ)
        self.pwm.start(0)

        
        rospy.init_node('led_listener', anonymous=True)
        rospy.Subscriber("/"+self._glz_name+"/base/move", Int32MultiArray, self.led_callback)

        GPIO.output(self._pin_config["EA_PIN"],GPIO.HIGH)

        rospy.loginfo(self._pin_config)

        while(True):
            self.setLED([0,1,2,3])

        rospy.spin()

        self.pwm.stop()


        GPIO.cleanup()

    def led_callback(self, data):
        # rospy.loginfo(data)
        # TODO: base moter control
        x = data.linear.x
        yaw = data.angular.y



    def setLED(self,arr):
        GPIO.output(self._pin_config["EA_PIN"],GPIO.LOW)
        for i in arr:
            self.pwm.ChangeDutyCycle(i/3*100)
            time.sleep(self.delay/1000)

        GPIO.output(self._pin_config["EA_PIN"],GPIO.HIGH)
        time.sleep(self.delay/1000)

        
            






if __name__ == '__main__':
    try:
        led = LED()
    except rospy.ROSInterruptException:
        pass
