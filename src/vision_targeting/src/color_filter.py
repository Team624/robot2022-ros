import cv2
import numpy as np
import rospy
class ColorFilter:

    def __init__(self, frame):
        self.frame = frame
        self.frame = cv2.GaussianBlur(self.frame, (3,3), 0)

    def convertToGreen(self):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lower = np.array(rospy.get_param("~lower_hsv", [34.32, 99.91, 107.88]))
        upper = np.array(rospy.get_param("~higher_hsv", [95.625, 255, 255]))
        mask = cv2.inRange(hsv, lower, upper)    
        return mask