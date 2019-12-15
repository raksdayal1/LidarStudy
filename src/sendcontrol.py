#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi

# Around school
'''
Waypoints = [(23,10),
             (30,15),
             (30,40),
             (0, 50),
             (-25,30),
             (-15,10),
             (0,10)]
'''

Waypoints = [(30,0),
             (30,30),
             (0,30),
             (-10, 30),
             (-10,0),
             (0,0)]



def main():
    # Initialize the ros node
    rospy.init_node('ControlNode', anonymous=True)

    # create a publisher to send twist message to the pioneer robot
    vel_pub = rospy.Publisher('/pioneer2dxLidar/DiffDriveControlCmd', Twist, queue_size=10)

    # create a tf listener
    listener = tf.TransformListener()

    # Set the rate
    rate = rospy.Rate(100)

    goal = Point()
    vel_msg = Twist()

    Wp_Counter = 0


    while True:
        Curr_WP = Waypoints[Wp_Counter]
        goal.x = Curr_WP[0]
        goal.y = Curr_WP[1]
        # Listen to the TransformListener to get the odom to chassis transform
        try:
            (trans, rot) = listener.lookupTransform('odom','chassis', rospy.Time(0))
        except:
            continue

        (roll, pitch, yaw) = euler_from_quaternion(rot)

        x = trans[0]
        y = trans[1]
        theta = yaw


        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)


        if abs(angle_to_goal - theta)*180/pi > 10:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.6
        else:
            vel_msg.linear.x = 12
            vel_msg.angular.z = 0.0

        if (sqrt((goal.x-x)**2+(goal.y-y)**2) < 3):
            print("Finished waypoint %d" % Wp_Counter)
            Wp_Counter += 1
            print("Entering waypoint %d" % Wp_Counter)
            print("-----------------------------------")
            if(Wp_Counter > len(Waypoints)-1):
                Wp_Counter = 0


        vel_pub.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
