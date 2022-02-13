#!/usr/bin/env python

import numpy as np
import cv2
import math
import ds

from matplotlib import pyplot as plt
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64, Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import color_filter
import contour_detection
import ds


rospy.init_node('vision_ball', anonymous=True)
class VisionBall:
    def __init__(self):
        self.bridge = CvBridge()
        

        # Sending values
        self.rotation_pub = rospy.Publisher("/ball/rotationAngle", Float32, queue_size=1)
        self.ball_pub = rospy.Publisher("/ball/ballArea", Float32, queue_size=1)


        self.cap = cv2.VideoCapture(2)

        self.rotationAngle = ds.Window(maxLen=rospy.get_param("~rotation_window_size", 5))
        self.ballArea = ds.Window(maxLen=rospy.get_param("~ball_window_size", 5))
        
    
        # cv2.imshow('image', mask)
    def main(self):
        try:
            r = rospy.Rate(rospy.get_param("~rate", 50))
            while not rospy.is_shutdown():
                success, img = self.cap.read()
                height, width, _ = img.shape
                    
                center_x, center_y = width/2, height/2
                
                color = color_filter.ColorFilter(img)
                mask = color.colorFilter(rospy.get_param("lower_hsv",[90, 175, 0]),rospy.get_param("higher_hsv",[140, 255, 255]))
                m = contour_detection.ContourDetection(mask)
                contour = m.getContours(mask,rospy.get_param("~minArea",150)),rospy.get_param("~e",0.5)
                center = m.getContoursCenter(contour)

                self.rotationAngle=180
                
                if center is not None:
                    center[0]=int(center[0])
                    center[1]=int(center[1])
                    center=(center[0], center[1])
                    img=cv2.circle(img, center, 10, (0,0,255),-1)
                    rot_angle = contour_detection.getAngle(center, center_x, height, width)
                self.rotationAngle.add(rot_angle)
                self.ballArea.add(cv2.contourArea)
                rot_data = Float32()
                rot_data.data = self.rotationAngle.getAverage()

                ball_data = Float32()
                ball_data.data = contour_detection.getBallArea(contour)

                self.rotation_pub.publish(rot_data)
                self.ball_pub.publish(ball_data)

                # Sleeps to meet specified rate
                r.sleep()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()

if __name__ == '__main__':
    dvt = VisionBall()
    dvt.main()