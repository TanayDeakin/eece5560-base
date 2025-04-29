#!/usr/bin/env python3

# Python Libs
import sys, time
import numpy as np
import cv2
from cv_bridge import CvBridge

# ROS Libraries
import rospy
import roslib

# ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()
        
        # Change this to match your robot's camera topic
        self.image_sub = rospy.Subscriber('/robot/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        rospy.init_node("my_lane_detector", anonymous=True)

    def image_callback(self, msg):
        rospy.loginfo("Processing frame...")

        # Convert to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Crop image (rough crop: remove top 2/3)
        height, width, _ = img.shape
        cropped_img = img[int(height/2):, :]

        # Convert to HSV
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # --- Filter White pixels ---
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv_img, white_lower, white_upper)
        white_result = cv2.bitwise_and(cropped_img, cropped_img, mask=white_mask)
        cv2.imshow("White Filtered", white_result)

        # --- Filter Yellow pixels ---
        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
        yellow_result = cv2.bitwise_and(cropped_img, cropped_img, mask=yellow_mask)
        cv2.imshow("Yellow Filtered", yellow_result)

        # --- Canny Edge Detection (for visualization only) ---
        edges = cv2.Canny(cropped_img, 50, 150)

        # --- Hough Transform on white lines ---
        white_gray = cv2.cvtColor(white_result, cv2.COLOR_BGR2GRAY)
        white_lines = cv2.HoughLinesP(white_gray, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)

        # --- Hough Transform on yellow lines ---
        yellow_gray = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)
        yellow_lines = cv2.HoughLinesP(yellow_gray, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)

        # --- Draw Lines ---
        output_img = cropped_img.copy()
        if white_lines is not None:
            for line in white_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output_img, (x1, y1), (x2, y2), (255, 255, 255), 2)

        if yellow_lines is not None:
            for line in yellow_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 255), 2)

        cv2.imshow("Hough Lines Output", output_img)

        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
