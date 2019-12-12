#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *


def main():
    rospy.init_node('ControlNode', anonymous=True)
    vel_pub = rospy.Publisher('/pioneer2dxLidar/DiffDriveControlCmd', Twist, queue_size=10)
    vel_msg = Twist()

    rate = rospy.Rate(100)
    vel_msg.linear.x = 10
    vel_msg.angular.z = -0.5

    while True:
        vel_pub.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
