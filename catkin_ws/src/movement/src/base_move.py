#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class BaseMove():
    def __init__(self):
        
        rospy.init_node('move_listener', anonymous=True)
        rospy.Subscriber("base/move", Twist, self._move_callback)

        rospy.spin()
    
    def _move_callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + data.data)
        # TODO: base moter control

    

if __name__ == '__main__':  
    try:
        bm = BaseMove()
    except rospy.ROSInterruptException:
        pass