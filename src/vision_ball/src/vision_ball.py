#!/usr/bin/env python

import numpy as np
import cv2
import math
import numpy as np
import rospy
import ds
from std_msgs.msg import Float32, Float64, Bool



rospy.init_node('vision_ball', anonymous=True)
class VisionBall:
    def __init__(self):

        # Sending values
        self.rotation_pub = rospy.Publisher("/ball/rotationAngle", Float32, queue_size=1)
        self.ball_pub = rospy.Publisher("/ball/ballArea", Float32, queue_size=1)

        rospy.Subscriber("/ball/isBlue", Bool, self.get_is_blue)


        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920/3)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080/3)

        self.rotationAngle = ds.Window(maxLen=rospy.get_param("~rotation_window_size", 3))
        self.ballArea = ds.Window(maxLen=rospy.get_param("~ball_window_size", 3))

        self.lower_b = rospy.get_param("~lower_b", [90, 134, 0])
        self.higher_b = rospy.get_param("~higher_b", [140, 255, 255])

        self.lower_r = rospy.get_param("~lower_r", [90, 134, 0])
        self.higher_r = rospy.get_param("~higher_r", [140, 255, 255])

        self.is_blue = True

    def get_x_offset(self, x, wanted_x):
        return x - (wanted_x)

    def getAngle(self, center, center_x, height, width):
        angle = (math.atan2(self.get_x_offset(center[0], center_x),  height))*180/math.pi % 360.0
        if angle>180:
            angle= 360-angle
        if center[0]>=width/2:
            return angle
        else:
            return -angle
        
    def colorFilter(self, img,lower, upper):
        blurred = cv2.GaussianBlur(img,(17,17),0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper) 
        x=5
        kernel=np.ones((x,x),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def getContours(self, mask, minArea, e):
        edges = cv2.Canny(mask,100, 200)
        _, contours, _= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        newContours = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, .03*cv2.arcLength(contour, True), True)
            eccen = self.eccentricity(contour)<e
            if cv2.contourArea(contour)>minArea and eccen and len(approx)>6:
                newContours.append(contour)
        if len(newContours)==0:
            return None
        newContours=sorted(newContours, key=cv2.contourArea, reverse=False)
        return newContours[len(newContours)-1]

    def getContoursCenter(self, contour):
        x1=0
        y1=0
        M = cv2.moments(contour)
        if M["m00"] !=0:
            x1 = int(M["m10"] / M["m00"])
            y1 = int(M["m01"] / M["m00"])
        else:
            return None
        return [x1, y1]

    def eccentricity(self, contour):
        try:
            (_, _), (MA, ma), _ = cv2.fitEllipse(contour)
            a = ma / 2
            b = MA / 2
            ecc = np.sqrt(a ** 2 - b ** 2) / a
        except Exception:
            ecc=1
        return ecc

    def empty(self,a):
        pass
    

    def get_is_blue(self, msg):
        self.is_blue = msg.data
    
    def main(self):
        try:
            r = rospy.Rate(rospy.get_param("~rate", 50))
            while not rospy.is_shutdown():
                success, img = self.cap.read()

                if self.is_blue:
                    lower = np.array(self.lower_b)
                    upper = np.array(self.higher_b)
                else:
                    lower = np.array(self.lower_r)
                    upper = np.array(self.higher_r)

                mask = self.colorFilter(img,lower,upper)

                height, width, _ = img.shape
                    
                center_x, center_y = width/2, height/2
                
                contour = self.getContours(mask,150,0.65)
                center = self.getContoursCenter(contour)
                
                rot_data = Float32()
                ball_data = Float32()

                if center is not None:
                    center[0]=int(center[0])
                    center[1]=int(center[1])
                    center=(center[0], center[1])
                    img=cv2.circle(img, center, 10, (0,0,255),-1)
                    rot_angle = self.getAngle(center, center_x, height, width)
                    self.rotationAngle.add(rot_angle)
                    self.ballArea.add(cv2.contourArea(contour))

                    rot_data.data = self.rotationAngle.getAverage()

                    ball_data.data = self.ballArea.getAverage()
                else:
                    rot_data.data = -1000

                    ball_data.data = -1

                self.rotation_pub.publish(rot_data)
                self.ball_pub.publish(ball_data)

                # print("Angle: " + str(self.rotationAngle.getAverage()))
                # print("Area: " + str(self.ballArea.getAverage()))

                # cv2.imshow('image', img)
                # cv2.imshow('mask', mask)

                cv2.waitKey(3)

                r.sleep()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()

if __name__ == '__main__':
    dvt = VisionBall()
    dvt.main()