#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CannonMove():
    def __init__(self):

        self._glz_name = rospy.get_param('GLZ_NAME', 'GLZ00')
        self._pin_config = {"GPIO_YAW_PIN": rospy.get_param('~GPIO_YAW_PIN', 16),
                            "GPIO_PITCH_PIN": rospy.get_param('~GPIO_PITCH_PIN', 17)
                            }

        rospy.init_node('cannon_move_listener', anonymous=True)
        rospy.Subscriber(self._glz_name+"/base/cannon_move",
                         Twist, self._cannon_callback)

        rospy.spin()

    def _cannon_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + data.data)
        # TODO: cannon moter control


if __name__ == '__main__':
    try:
        bm = CannonMove()
    except rospy.ROSInterruptException:
        pass
