#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose

import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.goal_angle = 0
        self.goal_x = None
        self.goal_y = None
        self.dist_goal_active = False
        self.angle_goal_active = False
        self.position_goal_active = False
        self.current_pose = Pose()
        
        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Pose, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer (acts as the main function loop)
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

        rospy.loginfo("Initialized node!")
        rospy.spin()

    def pose_callback(self, msg):
        self.current_pose = msg
    
    def distance_callback(self, msg):
        self.last_distance = msg.data
    
    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        if self.goal_distance == 0:
            self.dist_goal_active = False
        else:
            self.dist_goal_active = True
    
    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        if self.goal_angle == 0:
            self.angle_goal_active = False
        else:
            self.angle_goal_active = True
    
    def goal_position_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.position_goal_active = True
    
    def timer_callback(self, event):
        cmd_vel = Twist()
        
        # Handle distance goal
        if self.dist_goal_active:
            if abs(self.last_distance) >= abs(self.goal_distance):
                self.dist_goal_active = False
            else:
                cmd_vel.linear.x = 1.0 if self.goal_distance > 0 else -1.0
        
        # Handle angle goal
        elif self.angle_goal_active:
            if abs(self.current_pose.theta - self.goal_angle) < 0.1:
                self.angle_goal_active = False
            else:
                cmd_vel.angular.z = 1.0 if self.goal_angle > 0 else -1.0
        
        # Handle position goal
        elif self.position_goal_active:
            dx = self.goal_x - self.current_pose.x
            dy = self.goal_y - self.current_pose.y
            distance_to_goal = math.sqrt(dx**2 + dy**2)
            
            if distance_to_goal < 0.1:
                self.position_goal_active = False
            else:
                desired_angle = math.atan2(dy, dx)
                angle_error = desired_angle - self.current_pose.theta
                
                if abs(angle_error) > 0.1:
                    cmd_vel.angular.z = 1.0 if angle_error > 0 else -1.0
                else:
                    cmd_vel.linear.x = 1.0
        
        self.velocity_publisher.publish(cmd_vel)

if __name__ == '__main__': 
    try: 
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
