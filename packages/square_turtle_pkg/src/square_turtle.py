#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)

    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Turtles are great at drawing squares!")

    # Create Twist message once to reuse
    vel_msg = Twist()

    while not rospy.is_shutdown():
        for _ in range(4):  # Four sides of a square

            # Move forward
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(1.5)  # Move forward for 1.5 seconds

            # Stop
            vel_msg.linear.x = 0.0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.5)

            # Rotate 90 degrees
            vel_msg.angular.z = 1.56  # ~90 degrees/sec
            velocity_publisher.publish(vel_msg)
            rospy.sleep(1.0)  # Turn for 1 second

            # Stop turning
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass