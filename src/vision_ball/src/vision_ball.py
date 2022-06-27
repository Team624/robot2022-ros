#!/usr/bin/env python

import cv2
import imutils
import math

class VisionBall:
    
    def __init__(self, isBlue):
        
        self.cap = cv2.VideoCapture(0)

        #hsv values
        if isBlue:
            self.lower = (90, 175, 0)
            self.upper = (140, 255, 255)
        else:
            self.lower = (160,50,50)
            self.upper = (180,255,255)

        #width and height of the kernel
        #must be odd
        self.blur_constant = 13
        #min radius of a circle for it to be displayed
        self.min_radius = 20
    
    def get_x_offset(self, x, wanted_x):
	    return x - wanted_x

    #for future use
    def getAngle(self, contour, img):
        height, width, _ = img.shape
        M = cv2.moments(contour)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        angle = (math.atan2(self.get_x_offset(center[0], width/2),  height))*180/math.pi % 360.0
        if angle>180:
            angle= 360-angle
        if center[0]>=width/2:
            return angle
        else:
            return -angle

    def main(self):
        try:
            while True:
                _, frame = self.cap.read()
                if frame is None:
                    break
                frame = self.getImage(frame)
                cv2.imshow("Frame", frame)
                cv2.waitKey(3)
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()
    
    #image with balls circled
    def getImage(self, frame):
        blurred = cv2.GaussianBlur(frame, (self.blur_constant, self.blur_constant), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        for c in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > self.min_radius:
                frame = cv2.circle(frame, (int(x), int(y)), int(radius),(0,255,0), 10)
        return frame

if __name__ == '__main__':
    dvt = VisionBall(isBlue=True)
    dvt.main()

