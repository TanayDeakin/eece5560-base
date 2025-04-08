##!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
from duckietown_msgs.msg import FSMState
import math

class ClosedLoopController:
    def __init__(self):
        rospy.init_node("closed_loop_square_node")

        self.cmd_pub = rospy.Publisher('/robot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/robot/fsm_node/mode', FSMState, self.fsm_callback)
        rospy.Subscriber('/robot/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/robot/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)

        self.cmd_msg = Twist2DStamped()
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        self.ticks_per_meter = 1350  # Adjust based on your robot calibration
        self.ticks_per_90_deg = 300  # Adjust based on your robot's turn arc

        self.moving = False
        self.goal_ticks = 0
        self.movement_type = None  # "straight" or "rotate"

    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.draw_closed_loop_square()

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def reset_encoders(self):
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

    def get_average_ticks(self):
        delta_left = abs(self.left_ticks - self.last_left_ticks)
        delta_right = abs(self.right_ticks - self.last_right_ticks)
        return (delta_left + delta_right) / 2.0

    def move_straight(self, distance, speed):
        direction = 1 if distance > 0 else -1
        self.reset_encoders()
        self.goal_ticks = abs(distance) * self.ticks_per_meter
        self.movement_type = "straight"
        self.moving = True

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.moving:
            avg_ticks = self.get_average_ticks()
            if avg_ticks >= self.goal_ticks:
                break

            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = speed * direction
            self.cmd_msg.omega = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()

    def rotate_in_place(self, angle_deg, angular_speed):
        direction = 1 if angle_deg > 0 else -1
        self.reset_encoders()
        self.goal_ticks = abs(angle_deg) / 90.0 * self.ticks_per_90_deg
        self.movement_type = "rotate"
        self.moving = True

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.moving:
            avg_ticks = self.get_average_ticks()
            if avg_ticks >= self.goal_ticks:
                break

            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed * direction
            self.cmd_pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.cmd_pub.publish(self.cmd_msg)
        self.moving = False
        rospy.sleep(1)

    def draw_closed_loop_square(self):
        side_length = 1.0  # meters
        linear_speed = 0.3
        angular_speed = 1.0

        for i in range(4):
            rospy.loginfo(f"Side {i+1}")
            self.move_straight(side_length, linear_speed)
            rospy.sleep(1)
            self.rotate_in_place(90, angular_speed)
            rospy.sleep(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ClosedLoopController()
        controller.run()
    except rospy.ROSInterruptException:
        pass