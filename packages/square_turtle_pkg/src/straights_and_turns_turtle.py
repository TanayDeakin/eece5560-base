#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        self.current_pose = Pose()
        self.goal_distance = None
        self.goal_angle = None
        self.goal_position = None
        self.start_pose = None
        self.moving = False
        self.position_stage = "rotate"

        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.3), self.timer_callback)

        rospy.loginfo("Initialized node!")

        rospy.spin()

    def pose_callback(self, msg):
        self.current_pose = msg

    def distance_callback(self, msg):
        pass  # Not needed for position goal handling

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.start_pose = self.current_pose
        self.moving = True

    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        self.start_angle = self.current_pose.theta
        self.moving = True

    def goal_position_callback(self, msg):
        self.goal_position = msg
        self.moving = True
        self.position_stage = "rotate"

    def timer_callback(self, event):
        if not self.moving:
            return

        vel_msg = Twist()

        # Distance Goal
        if self.goal_distance is not None:
            distance_traveled = self.calculate_distance(self.start_pose, self.current_pose)
            if abs(distance_traveled) < abs(self.goal_distance) - 0.05:
                vel_msg.linear.x = 1.0 if self.goal_distance > 0 else -1.0
            else:
                vel_msg.linear.x = 0.0
                self.goal_distance = None
                self.moving = False
                self.velocity_publisher.publish(vel_msg)  # Ensure stopping

        # Angle Goal
        elif self.goal_angle is not None:
            angle_traveled = self.normalize_angle(self.current_pose.theta - self.start_angle)
            if abs(angle_traveled) < abs(self.goal_angle) - 0.05:
                vel_msg.angular.z = 1.0 if self.goal_angle > 0 else -1.0
            else:
                vel_msg.angular.z = 0.0
                self.goal_angle = None
                self.moving = False
                self.velocity_publisher.publish(vel_msg)  # Ensure stopping

        # Position Goal
        elif self.goal_position is not None:
            target_x, target_y = self.goal_position.x, self.goal_position.y
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            target_angle = math.atan2(dy, dx)
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

            if self.position_stage == "rotate":
                angle_error = self.normalize_angle(target_angle - self.current_pose.theta)
                if abs(angle_error) > 0.5:
                    vel_msg.angular.z = 1.0 if angle_error > 0 else -1.0
                else:
                    vel_msg.angular.z = 0.0
                    self.position_stage = "move"

            elif self.position_stage == "move":
                if distance_to_target > 0.35:
                    vel_msg.linear.x = 1.0
                else:
                    vel_msg.linear.x = 0.0
                    self.goal_position = None
                    self.moving = False
                    self.velocity_publisher.publish(vel_msg)  # Ensure stopping

        self.velocity_publisher.publish(vel_msg)

    def calculate_distance(self, start_pose, current_pose):
        return math.sqrt((current_pose.x - start_pose.x) ** 2 + (current_pose.y - start_pose.y) ** 2)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass