#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
 
class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        
        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/robot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/robot/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

        self.old_state = None
        
    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if self.old_state == msg.state:
            rospy.loginfo("Duplicate keypress ignored")
            return
        self.old_state = msg.state
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.move_robot()
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()  # keeps node from exiting until node has shutdown

    # Robot drives in a square and then stops
    def move_robot(self):
        # Move forward for 1 meter (2 seconds)
        for i in range(4):
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.25  # Forward velocity (0.5 m/s)
            self.cmd_msg.omega = 0.0  # No rotation
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Moving forward!")
            rospy.sleep(4)  # Move for 2 seconds (1 meter)
            
            # Turn 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0  # Stop moving forward
            self.cmd_msg.omega = 1.57  # Set angular velocity to turn 90 degrees (1.57 rad/s = 90 degrees/s)
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning 90 degrees!")
            rospy.sleep(1)  # Turn for 1 second (90 degrees)
        
        # Stop robot after completing the square
        self.stop_robot()
        

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
