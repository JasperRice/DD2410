#!/usr/bin/env python
import rospy
from ras_lab1_msgs.msg import Encoders
from ras_lab1_msgs.msg import PWM
from geometry_msgs.msg import Twist
from std_msgs.msg import *

ticks_per_rev = 360.0
b = 0.115
r = 0.0352
f = 10.0
pi = 3.14159265358979323
alpha = 25.0

pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
pwm_header = std_msgs.msg.Header()
pwm_header.stamp.secs = 0

desired_linear_velocity = 0.3
desired_angular_velocity = 0.0
desired_wheel_linear_velocity1 = desired_linear_velocity - b * desired_angular_velocity
desired_wheel_linear_velocity2 = desired_linear_velocity + b * desired_angular_velocity
pwm1 = desired_linear_velocity * 465.0
pwm2 = desired_linear_velocity * 425.0

def callback_encoders(data):
    global pwm1, pwm2
    delta_encoder1 = float(data.delta_encoder1)
    delta_encoder2 = float(data.delta_encoder2)
    estimated_wheel_linear_velocity1 = 2.0 * pi * r * f * delta_encoder1 / ticks_per_rev
    estimated_wheel_linear_velocity2 = 2.0 * pi * r * f * delta_encoder2 / ticks_per_rev
    error1 = desired_wheel_linear_velocity1 - estimated_wheel_linear_velocity1
    error2 = desired_wheel_linear_velocity2 - estimated_wheel_linear_velocity2
    pwm1 = pwm1 + alpha * error1
    pwm2 = pwm2 + alpha * error2
    pub.publish(pwm_header, pwm1, pwm2)
    '''
    print("desired_wheel_linear_velocity1", desired_wheel_linear_velocity1)
    print("desired wheel_linear_velocity2", desired_wheel_linear_velocity2)
    print("estimated_wheel_linear_velocity1",estimated_wheel_linear_velocity1)
    print("estimated_wheel_linear_velocity2", estimated_wheel_linear_velocity2)
    print("estimated_linear_velocity", (estimated_wheel_linear_velocity1 + estimated_wheel_linear_velocity2) / 2.0)
    print("estimated_angular_velocity", (estimated_wheel_linear_velocity2 - estimated_wheel_linear_velocity1) / (2.0 * b))
    print("PWM 1", pwm1, "PWM 2", pwm2)
    '''

def encoder_subscriber():
    rospy.init_node('encoder_subscriber', anonymous=True)
    rospy.Subscriber('/kobuki/encoders', Encoders, callback_encoders)
    # inital movement
    pub.publish(pwm_header, pwm1, pwm2)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    encoder_subscriber()
