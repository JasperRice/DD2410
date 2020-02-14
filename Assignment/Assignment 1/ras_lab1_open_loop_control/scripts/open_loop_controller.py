#!/usr/bin/env python
import rospy
from ras_lab1_msgs.msg import PWM
from ras_lab1_msgs.msg import Encoders
from std_msgs.msg import *

def open_loop_controller():
    pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
    rospy.init_node('open_loop_controller', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        open_loop_controller_header = std_msgs.msg.Header()
        # open_loop_controller_header.seq = ?
        open_loop_controller_header.stamp.secs = 0
        # open_loop_controller_header.frame_id = ?
        open_loop_controller_PWM1 = 255
        open_loop_controller_PWM2 = 255
        pub.publish(open_loop_controller_header, open_loop_controller_PWM1, open_loop_controller_PWM2)
        rate.sleep()

if __name__ == '__main__':
    try:
        open_loop_controller()
    except rospy.ROSInterruptException:
        pass
