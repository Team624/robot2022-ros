#!/usr/bin/env python

import numpy as np
import cv2
import math

import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64, Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('vision_ball_targeting', anonymous=True)

class BallDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.feedback_pub = rospy.Publisher("auto/select", Float32, queue_size=1)

        rospy.Subscriber("/auto/state", Bool, self.is_auto)

        #lowest contour area for it to be a red ball
        self.pathColorMin = 1400

        self.red = True

        self.is_auto = False

    def is_auto(self, data):
        self.is_auto = data.data

    def pixelDistance(self,x1,y1,x2,y2):
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return dist

    def xDistanceFromCenter(self,x, width):
        return (x - width)

    def getLeftCount(self,boxes, width):
        leftCount = 0

        for box in boxes:
            if self.xDistanceFromCenter(box[1][0], width) < 0:
                leftCount+=1
        
        return leftCount

    def isRed(self,box):
        if cv2.contourArea(box) > self.pathColorMin:
            return True
        else:
            return False

    #pathIndex 7 = redA 8 = redB 9 = blueA 10 = blueB
    def pathDetection(self,boxes, width):
        self.red = True
        pathIndex = 0

        if len(boxes) != 0:
            self.red = self.isRed(boxes[0])

        leftCount = self.getLeftCount(boxes, width)
        print(leftCount)
        
        if len(boxes) != 0:
            if self.red:
                if self.xDistanceFromCenter(boxes[0][1][0], width/2) > width/2 - 100 and self.xDistanceFromCenter(boxes[0][1][0], width/2) < width/2 + 100:
                    pathIndex = 7
                else:
                    pathIndex = 8
            else:
                if leftCount == 1:
                    pathIndex = 10
                else:
                    pathIndex = 9

        return pathIndex

    

    def prime_callback(self,data):
        #This lets the node know if it should be tracking 
        self.is_tracking = data

    def main (self):
        try:
            r = rospy.Rate(20)

            cap = cv2.VideoCapture(0)

            kernel = np.ones((2, 2), 'uint8')
            font = cv2.FONT_HERSHEY_SIMPLEX
            org = (0, 20)
            fontScale = 0.6
            color = (0, 0, 255)
            thickness = 2
        
            #cv2.namedWindow('Object Dist Measure ', cv2.WINDOW_NORMAL)
            #cv2.resizeWindow('Object Dist Measure ', 700, 600)

            minContSize = 100
            maxContSize = 4000
            while not rospy.is_shutdown():
                ret, img = cap.read()

                hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                # predefined mask for yellow colour detection
                #lower = np.array([20, 103, 102])
                #upper = np.array([43, 239, 250])
                lower = np.array([20, 100, 100])
                upper = np.array([30, 255, 255])

                mask = cv2.inRange(hsv_img, lower, upper)

                #cv2.imshow("mask", mask)

                # clean up image
                d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=5)

                #cv2.imshow("ig", d_img)

                # find the histogram
                cont, hei = cv2.findContours(d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
                cont = sorted(cont, key=cv2.contourArea, reverse=True)

                boxes = []
                rects = []
                contours = []

                fwidth = img.shape[1]
                fheight = img.shape[0]

                for cnt in cont:
                    # check for contour area    
                    if (cv2.contourArea(cnt) > minContSize and cv2.contourArea(cnt) < maxContSize):

                        # Draw a rectangle on the contour
                        rect = cv2.minAreaRect(cnt)
                        box = cv2.boxPoints(rect)
                        
                        box = np.int0(box)
                        
                        if box[1][1] > fheight / 2 - 100:
                            rects.append(rect)
                            boxes.append(box)
                        #cv2.drawContours(img, [cnt], -1, (255, 0, 0), 3)

                        #img = get_dist(rect, img)
                        
                
                
                img = cv2.putText(img, 'Count ' + str(len(boxes)), org, font, 1, color, 2, cv2.LINE_AA)
                img = cv2.putText(img, 'Left ' + str(self.getLeftCount(boxes, int(fwidth/2))), (0, 40),  font, 1, color, 2, cv2.LINE_AA)
                img = cv2.putText(img, 'Path ' + str(self.pathDetection(boxes, int(fwidth/2))), (0, 60),  font, 1, color, 2, cv2.LINE_AA)
                img = cv2.putText(img, 'Red ' + str(self.isRed(boxes[0]) if len(boxes) != 0 else "N/A"), (0, 80),  font, 1, color, 2, cv2.LINE_AA)
                
                #print("width: " + str(fwidth) + " height: " + str(fheight))
                img = cv2.circle(img, (int(fwidth/2), int(fheight/2)), 5, color, 2)

                for rect in rects:
                    box = cv2.boxPoints(rect)
                    
                    box = np.int0(box)

                    area = cv2.contourArea(box)

                
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    #print(rect)
                    #print(box)
                    #print(area)
                    img = cv2.putText(img, "area " + str(area), (box[1][0], box[1][1]), font, 1, color, 2, 1)
                    img = cv2.line(img, (box[1][0], box[1][1]), (int(fwidth/2), int(fheight/2)), color, 2) 

                    #distance = pixelDistance(box[1][0], box[1][1], int(fwidth/2), int(fheight/2))
                    #img = cv2.putText(img, str(distance), (box[1][0], box[1][1] + 40), font, 1, color, 2, 2)

                    xDistance = self.xDistanceFromCenter(box[1][0], int(fwidth/2))
                    img = cv2.putText(img, "x distance " + str(xDistance), (box[1][0], box[1][1] + 40), font, 1, color, 2, 2)

                    xDistance = self.xDistanceFromCenter(box[1][0], int(fwidth/2))
                    img = cv2.putText(img, "y value " + str(box[1][1]), (box[1][0], box[1][1] + 60), font, 1, color, 2, 2)
                    
                selectedPath = self.pathDetection(boxes, int(fwidth/2))

                rospy.loginfo(selectedPath)
                if (self.is_auto != True):
                    self.feedback_pub.publish(selectedPath)

                # cv2.imshow('Object Dist Measure ', img)
                # cv2.imshow('Mask ', mask)
                #except KeyboardInterrupt:
                #print("Shutting down")

                cv2.waitKey(3)
                # Sleeps to meet specified rate
                r.sleep()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
                
if __name__ == '__main__':
    dvt = BallDetector()
    dvt.main()

