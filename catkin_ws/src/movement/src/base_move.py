#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class BaseMove():
    def __init__(self):
        self._glz_name = rospy.get_param('GLZ_NAME', 'GLZ00')
        self._pin_config = {"GPIO_LF_PIN": rospy.get_param('~GPIO_LF_PIN',10),
                            "GPIO_LR_PIN": rospy.get_param('~GPIO_LR_PIN',11),
                            "GPIO_LPWM_PIN": rospy.get_param('~GPIO_LPWM_PIN',12),
                            "GPIO_RF_PIN": rospy.get_param('~GPIO_RF_PIN',13),
                            "GPIO_RR_PIN": rospy.get_param('~GPIO_RR_PIN',14),
                            "GPIO_RPWM_PIN": rospy.get_param('~GPIO_RPWM_PIN',15)}
        
        rospy.init_node('base_move_listener', anonymous=True)
        rospy.Subscriber(self._glz_name+"/base/move", Twist, self._move_callback)

        rospy.loginfo(self._pin_config)

        rospy.spin()

    def _move_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + data.data)
        # TODO: base moter control


if __name__ == '__main__':
    try:
        bm = BaseMove()
    except rospy.ROSInterruptException:
        pass
