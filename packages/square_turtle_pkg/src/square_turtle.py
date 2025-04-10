# #!/usr/bin/env python3
# # Import Dependencies
# import rospy 
# from geometry_msgs.msg import Twist 
# import time 

# def move_turtle_square(): 
#     rospy.init_node('turtlesim_square_node', anonymous=True)
    
#     # Init publisher
#     velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
#     rospy.loginfo("Turtles are great at drawing squares!")

#     ########## YOUR CODE GOES HERE ##########
#     while not rospy.is_shutdown():
        
#         rate = rospy.Rate(1)  # 1 Hz, adjust as needed
        
#         # Create a Twist message for moving forward 
#         cmd_vel_msg = Twist() 
#         cmd_vel_msg.linear.x = 2.0  # Linear velocity
#         velocity_publisher.publish(cmd_vel_msg) # Publish!

#         rate.sleep()

#         # Create a Twist message for moving forward 
#         cmd_vel_msg = Twist() 
#         cmd_vel_msg.linear.y = 2.0  # Linear velocity
#         velocity_publisher.publish(cmd_vel_msg) # Publish!

#         rate.sleep()

#         # Create a Twist message for moving backward 
#         cmd_vel_msg = Twist()
#         cmd_vel_msg.linear.x = -2.0  # Linear velocity
#         velocity_publisher.publish(cmd_vel_msg) # Publish!

#         rate.sleep()

#         # Create a Twist message for moving forward 
#         cmd_vel_msg = Twist() 
#         cmd_vel_msg.linear.y = -2.0  # Linear velocity
#         velocity_publisher.publish(cmd_vel_msg) # Publish!

#         rate.sleep()

#         ###########################################

# if __name__ == '__main__': 

#     try: 
#         move_turtle_square() 
#     except rospy.ROSInterruptException: 
#         pass

#!/usr/bin/env python3

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)

    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Turtle is drawing squares!")

    # Create a Twist message
    move_cmd = Twist()
    turn_cmd = Twist()

    # Parameters (adjust for smoother turns or size of square)
    forward_speed = 1.0   # Linear velocity
    turn_speed = 1.0      # Angular velocity
    forward_time = 2.0    # Seconds to move forward
    turn_time = 1.57      # Approx time to turn 90 degrees (pi/2 radians at 1 rad/s)

    while not rospy.is_shutdown():
        for _ in range(4):  # Draw one square
            # Move forward
            move_cmd.linear.x = forward_speed
            move_cmd.angular.z = 0.0
            velocity_publisher.publish(move_cmd)
            time.sleep(forward_time)

            # Stop
            move_cmd.linear.x = 0.0
            velocity_publisher.publish(move_cmd)
            time.sleep(0.5)

            # Turn
            turn_cmd.linear.x = 0.0
            turn_cmd.angular.z = turn_speed
            velocity_publisher.publish(turn_cmd)
            time.sleep(turn_time)

            # Stop
            turn_cmd.angular.z = 0.0
            velocity_publisher.publish(turn_cmd)
            time.sleep(0.5)

if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass