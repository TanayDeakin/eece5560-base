#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/robot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/robot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        self.tag_detected = False  # To keep track of current state (object found or not)
        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if len(detections) == 0:
            # No tag detected: SEEK MODE
            rospy.loginfo("No object detected. Seeking...")
            cmd_msg.v = 0.0
            cmd_msg.omega = 1.0  # Rotate in place
            self.tag_detected = False
        else:
            # Tag detected: LOOK AT OBJECT
            tag = detections[0]
            x = tag.transform.translation.x
            rospy.loginfo("Object detected at x: %.2f", x)

            cmd_msg.v = 0.0

            # Determine angular velocity to center the tag
            if abs(x) < 0.05:
                cmd_msg.omega = 0.0  # Tag is centered
                rospy.loginfo("Object centered.")
            else:
                cmd_msg.omega = -1.0 * x  # Rotate proportionally to x offset
                rospy.loginfo("Adjusting to center object...")

            self.tag_detected = True

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass