import cv2
import math
import contour_detection
import rospy

class DataGatherer:
    def __init__(self, image):
        self.contours = contour_detection.ContourDetection(image).getContours()
        self.w, self.h, _=image.shape

    def getCenters(self):
        mainCountors = self.contours
        points = []
        for c in mainCountors:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            points.append((cX, cY, c))
        return points

    def getDistanceBetweenTwoPoints(self, p1, p2):
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        return abs((x1-x2)**2+(y1-y2)**2)**.5
    def getAverageCenter(self):
        self.contours = self.getBestContours(rospy.get_param("~tolerance", 40))
        if len(self.contours)<3:
            return None
        totalX=0
        totalY=0
        for (x,y,c) in self.getCenters():
            totalX=totalX+x
            totalY=totalY+y
        length = len(self.contours)
        return [totalX/length, totalY/length]

    def get_x_offset(self, x, wanted_x):
        return x - (wanted_x)

    def inRange(self, x, minX, maxX):
        return minX<=x and x<=maxX

    def getBestContours(self, tolerance):
        
        variation = self.h/tolerance
        data, newData = [], []
        centers = self.getCenters()
        for i in range(0, len(centers)):
            mainCntY = centers[i][1]
            for cnt in centers:
                if self.inRange(cnt[1], mainCntY-variation, mainCntY+variation):
                    newData.append(cnt[2])
            if len(newData)>len(data):
                data = newData
            newData=[]
        
        if len(data)<3:
            return []
        
        data = self.getCenterX(data)
        data = sorted(data, key = lambda x: x[0])
        distances = []
        for i in range(0,len(data)-1):
            distances.append(self.getDistanceBetweenTwoPoints(
                [data[i][0],0],
                [data[i+1][0], 0],
            ))
        average = sum(distances)/len(distances)
        maxdist = max(distances)
        if maxdist/average>1.4:
            index = distances.index(maxdist)
            if index>len(distances)/2:
                data = data[0:index+1]
            elif index<len(distances)/2:
                data=data[index+1:]
            else:
                return []
        for i in range(0, len(data)):

           data[i]=data[i][2]
        if len(data)<3:
            return [] 

        return data
        
    def getCenterX(self, contours):
        mainCountors = contours
        points = []
        for c in mainCountors:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            points.append((cX, cY, c))
        return points




    def getExtremes(self):
        data = self.getCenters()
        newData = []
        for i in range(0, len(data)):
            newData.append(
                [
                    data[i][0],
                    data[i][1],
                    self.contours[i]
                ]

        )
        data=newData
        data = sorted(data, key=lambda x2: x2[0])
        c=data[0][2]
        left = tuple(c[c[:, :, 0].argmin()][0])
        c=data[-1][2]
        right=tuple(c[c[:, :, 0].argmax()][0])
        return left, right

    def getAngle(self, averageCenter, center_x, height):
        angle = (math.atan2(self.get_x_offset(averageCenter[0], center_x),  height))*180/math.pi % 360.0
        if angle>180:
            return 360-angle
        else:
            return angle
    
    def getAngleAdvanced(self, averageCenter, center_x, height, width):
        angle = (math.atan2(self.get_x_offset(averageCenter[0], center_x),  height))*180/math.pi % 360.0
        if angle>180:
            angle= 360-angle
        if averageCenter[0]>=width/2:
            return angle
        else:
            return -angle