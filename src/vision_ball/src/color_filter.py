import cv2
import numpy as np
import rospy
class ColorFilter:
    def __init__(self, frame):
        self.frame = cv2.GaussianBlur(frame,(17,17),0)

    def colorFilter(self,lower, upper):
        
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        lower = np.array(rospy.get_param("~lower_hsv", [0,0,0]))
        upper = np.array(rospy.get_param("~higher_hsv", [0,0,0]))
        mask = cv2.inRange(hsv, lower, upper) 
        x=5
        kernel=np.ones((x,x),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask