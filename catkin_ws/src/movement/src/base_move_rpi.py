#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import math
import RPi.GPIO as GPIO


class BaseMove():
    def __init__(self):
        self._glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')
        self._pin_config = {"GPIO_LF_PIN": rospy.get_param('base_move_py/GPIO_LF_PIN',27),
                            "GPIO_LR_PIN": rospy.get_param('base_move_py/GPIO_LR_PIN',22),
                            "GPIO_LPWM_PIN": rospy.get_param('base_move_py/GPIO_LPWM_PIN',17),
                            "GPIO_RF_PIN": rospy.get_param('base_move_py/GPIO_RF_PIN',10),
                            "GPIO_RR_PIN": rospy.get_param('base_move_py/GPIO_RR_PIN',9),
                            "GPIO_RPWM_PIN": rospy.get_param('base_move_py/GPIO_RPWM_PIN',11)}

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin_config["GPIO_LF_PIN"], GPIO.OUT)
        GPIO.setup(self._pin_config["GPIO_LR_PIN"], GPIO.OUT)
        GPIO.setup(self._pin_config["GPIO_LPWM_PIN"], GPIO.OUT)
        GPIO.setup(self._pin_config["GPIO_RF_PIN"], GPIO.OUT)
        GPIO.setup(self._pin_config["GPIO_RR_PIN"], GPIO.OUT)
        GPIO.setup(self._pin_config["GPIO_RPWM_PIN"], GPIO.OUT)


        self.PWM_FREQ = 1000
        self.lpwm = GPIO.PWM(self._pin_config["GPIO_LPWM_PIN"], self.PWM_FREQ)
        self.rpwm = GPIO.PWM(self._pin_config["GPIO_RPWM_PIN"], self.PWM_FREQ)
        self.lpwm.start(0)
        self.rpwm.start(0)

        
        rospy.init_node('base_move_listener', anonymous=True)
        rospy.Subscriber("/"+self._glz_name+"/base/move", Twist, self._move_callback)

        rospy.loginfo(self._pin_config)

        rospy.spin()

        self.lpwm.stop()
        self.rpwm.stop()

        GPIO.cleanup()

    def _move_callback(self, data):
        # rospy.loginfo(data)
        # TODO: base moter control
        x = data.linear.x
        yaw = data.angular.y


        if(x > 1):
            x = 1
        elif(x < -1):
            x = -1

        if(yaw > math.pi/2):
            yaw = math.pi/2
        elif(yaw < -math.pi/2):
            yaw = -math.pi/2

        if(x == 0):
            if(yaw == 0):
                GPIO.output(self._pin_config["GPIO_LF_PIN"],GPIO.LOW)
                GPIO.output(self._pin_config["GPIO_LR_PIN"],GPIO.LOW)
                GPIO.output(self._pin_config["GPIO_RF_PIN"],GPIO.LOW)
                GPIO.output(self._pin_config["GPIO_RR_PIN"],GPIO.LOW)
                self.lpwm.ChangeDutyCycle(0)
                self.rpwm.ChangeDutyCycle(0)
            elif(yaw > 0):
                GPIO.output(self._pin_config["GPIO_LF_PIN"],GPIO.HIGH)
                GPIO.output(self._pin_config["GPIO_LR_PIN"],GPIO.LOW)
                GPIO.output(self._pin_config["GPIO_RF_PIN"],GPIO.LOW)
                GPIO.output(self._pin_config["GPIO_RR_PIN"],GPIO.HIGH)
                self.lpwm.ChangeDutyCycle(yaw/(math.pi/2)*100)
                self.rpwm.ChangeDutyCycle(yaw/(math.pi/2)*100)
            elif(yaw < 0):
                GPIO.output(self._pin_config["GPIO_LF_PIN"],GPIO.LOW)
                GPIO.output(self._pin_config["GPIO_LR_PIN"],GPIO.HIGH)
                GPIO.output(self._pin_config["GPIO_RF_PIN"],GPIO.HIGH)
                GPIO.output(self._pin_config["GPIO_RR_PIN"],GPIO.LOW)
                self.lpwm.ChangeDutyCycle(-yaw/(math.pi/2)*100)
                self.rpwm.ChangeDutyCycle(-yaw/(math.pi/2)*100)
            
            
        elif(x > 0):
            GPIO.output(self._pin_config["GPIO_LF_PIN"],GPIO.HIGH)
            GPIO.output(self._pin_config["GPIO_LR_PIN"],GPIO.LOW)
            GPIO.output(self._pin_config["GPIO_RF_PIN"],GPIO.HIGH)
            GPIO.output(self._pin_config["GPIO_RR_PIN"],GPIO.LOW)

            ldc = abs(x*math.sin((yaw+math.pi/2)/2)*100+30)
            rdc = abs(x*math.cos((yaw+math.pi/2)/2)*100+30)

            if(ldc > 100):
                ldc = 100
            if(rdc > 100):
                rdc = 100

            self.lpwm.ChangeDutyCycle( ldc )
            self.rpwm.ChangeDutyCycle( rdc )

        elif(x < 0):
            GPIO.output(self._pin_config["GPIO_LF_PIN"],GPIO.LOW)
            GPIO.output(self._pin_config["GPIO_LR_PIN"],GPIO.HIGH)
            GPIO.output(self._pin_config["GPIO_RF_PIN"],GPIO.LOW)
            GPIO.output(self._pin_config["GPIO_RR_PIN"],GPIO.HIGH)
            
            ldc = abs(x*math.sin((yaw+math.pi/2)/2)*100-30) 
            rdc = abs(x*math.cos((yaw+math.pi/2)/2)*100-30)

            if(ldc > 100):
                ldc = 100
            if(rdc > 100):
                rdc = 100
            
            self.lpwm.ChangeDutyCycle( ldc )
            self.rpwm.ChangeDutyCycle( rdc )





if __name__ == '__main__':
    try:
        bm = BaseMove()
    except rospy.ROSInterruptException:
        pass
