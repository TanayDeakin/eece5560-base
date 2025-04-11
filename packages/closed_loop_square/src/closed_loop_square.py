#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState

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

        self.ticks_per_meter = 740
        self.ticks_per_90_deg = 44

        self.state = "IDLE"  # Can be: IDLE, MOVING_STRAIGHT, ROTATING, DONE
        self.current_side = 0
        self.goal_ticks = 0
        self.direction = 1

        self.linear_speed = 0.7
        self.angular_speed = 1.0
        self.side_length = 1.0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)  # 10Hz

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING" and self.state == "IDLE":
            rospy.loginfo("Starting closed-loop square path")
            self.start_side_movement()

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def reset_encoders(self):
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

    def get_average_delta_ticks(self):
        delta_left = abs(self.left_ticks - self.last_left_ticks)
        delta_right = abs(self.right_ticks - self.last_right_ticks)
        return (delta_left + delta_right) / 2.0

    def start_side_movement(self):
        if self.current_side < 4:
            rospy.loginfo(f"Moving straight: Side {self.current_side + 1}")
            self.reset_encoders()
            self.goal_ticks = self.side_length * self.ticks_per_meter
            self.direction = 1
            self.state = "MOVING_STRAIGHT"
        else:
            rospy.loginfo("Finished drawing the square!")
            self.state = "DONE"
            self.stop_robot()

    def start_rotation(self):
        rospy.loginfo(f"Rotating 90 degrees")
        self.reset_encoders()
        self.goal_ticks = self.ticks_per_90_deg
        self.direction = 1
        self.state = "ROTATING"

    def control_callback(self, event):
        if self.state == "MOVING_STRAIGHT":
            if self.get_average_delta_ticks() >= self.goal_ticks:
                self.stop_robot()
                self.start_side_movement()
                
                
            else:
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = self.linear_speed * self.direction
                self.cmd_msg.omega = 0.0
                self.cmd_pub.publish(self.cmd_msg)

        elif self.state == "ROTATING":
            if self.get_average_delta_ticks() >= self.goal_ticks:
                self.stop_robot()
                self.current_side += 1
                self.start_rotation()
              
            else:
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.0
                self.cmd_msg.omega = self.angular_speed * self.direction
                self.cmd_pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.cmd_pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ClosedLoopController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
