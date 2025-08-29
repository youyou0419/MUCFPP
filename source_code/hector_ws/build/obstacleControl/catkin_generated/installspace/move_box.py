#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import sin

def oscillate_box():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('box_oscillator', anonymous=True)
    rate = rospy.Rate(50)  # 提高频率使运动更平滑
    
    while not rospy.is_shutdown():
        cmd = Twist()
        cmd.linear.y = sin(rospy.get_time()*0.8) * 1 # 范围[-0.5, 0.5]
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    oscillate_box()