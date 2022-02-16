#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64, Bool

import data_gatherer
import color_filter
import ds

rospy.init_node('vision_targeting', anonymous = True)
class VisionTargeting:

  def __init__(self):
    # ROS img msg
    # self.image_sub = rospy.Subscriber("d435/color/image_raw",Image,self.callback)

    # self.prime_sub = rospy.Subscriber("turret/primed", Bool,self.prime_callback)
    
    # # Rotation PID values
    self.rotation_pub = rospy.Publisher("/vision/rotationAngle", Float32, queue_size = 1)
    self.distance_pub = rospy.Publisher("/vision/distanceAngle", Float32, queue_size = 1)

    # # This will be used to calculate hood angle and flywheel rpm
    # self.y_offset_pub = rospy.Publisher("turret/y_offset", Float32, queue_size=1)

    self.cap = cv2.VideoCapture(0)
    codec = 0X47504A4D
    #self.cap.set(cv2.CAP_PROP_FPS,30)
    #self.cap.set(cv2.CAP_PROP_FOURCC, codec)
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920 / 3)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 / 3)
    #self.cap.set(cv2.CAP_PROP_EXPOSURE,-96/11)
    self.dist = 0
    self.focal = 450
    self.pixels = 30
    self.width = 4
    self.distanceAngle = ds.Window(maxLen = rospy.get_param("~distance_window_size", 5))
    self.rotationAngle = ds.Window(maxLen = rospy.get_param("~rotation_window_size", 5))

    # When to publish 180 rotation to find target
    self.find_target_thresh = rospy.get_param("~find_target_thresh", 40)
    self.find_target_ind = 0 

  def get_dist(self, rectangle_params):
    # find no of pixels covered
    pixels = rectangle_params[1][0]
    # calculate distance
    dist = (self.width * self.focal) / pixels
    return dist

  def main(self):
    try:
      r = rospy.Rate(rospy.get_param("~rate", 50))
      while not rospy.is_shutdown():
        success, img = self.cap.read()
        height, width, _ = img.shape
        
        center_x, center_y = width/2, height/2
        mask = color_filter.ColorFilter(img).convertToGreen()
        dataCollector = data_gatherer.DataGatherer(mask)
        averageCenter = dataCollector.getAverageCenter()
        if averageCenter is not None:
            self.find_target_ind = 0
            
            #casting 
            averageCenter[0] = int(averageCenter[0])
            averageCenter[1] = int(averageCenter[1])
            averageCenter = (averageCenter[0], averageCenter[1])
            img=cv2.circle(img, averageCenter, 10, (0, 0, 255), -1)
            angle = dataCollector.getAngleAdvanced(averageCenter, center_x, height, width)
            self.rotationAngle.add(angle)
            left, right = dataCollector.getExtremes()
            leftAngle, rightAngle = dataCollector.getAngle(left, center_x, height), dataCollector.getAngle(right, center_x, height)
            if (left[0] > width / 2 and right [0] > width / 2) or (left[0] < width / 2 and right[0] < width / 2):
                self.distanceAngle.add(abs(leftAngle - rightAngle))
            else:
                self.distanceAngle.add(leftAngle + rightAngle)

            #print(str(self.rotationAngle.getAverage())+" : "+str(self.distanceAngle.getAverage()))

            rot_data = Float32()
            rot_data.data = self.rotationAngle.getAverage()

            distance_data = Float32()
            distance_data.data = self.distanceAngle.getAverage()

            #print(rot_data.data)
            self.rotation_pub.publish(rot_data)
            self.distance_pub.publish(distance_data)
        else:
            self.find_target_ind += 1
            if (self.find_target_ind > self.find_target_thresh):
              rot_data = Float32()
              # Means no target found
              rot_data.data = 1000
              self.rotation_pub.publish(rot_data)

        # cv2.imshow('Contour Drawing', mask)
        # cv2.imshow('Image', img)

        cv2.waitKey(3)
        # Sleeps to meet specified rate
        r.sleep()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    dvt = VisionTargeting()
    dvt.main()
