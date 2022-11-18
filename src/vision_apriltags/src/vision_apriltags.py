#!/bin/env python3

import cv2
import apriltag as apriltag
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64, Bool
import json

rospy.init_node('vision_apriltags', anonymous=True)


class VisionApriltags:
    def __init__(self):
        self.rotation_pub = rospy.Publisher(
            "/apriltags/pose_est", Float32, queue_size=1)

        video_dev = int(rospy.get_param("~video_dev", 0))
        self.cap = cv2.VideoCapture(video_dev + cv2.CAP_V4L)
        

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30.0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 50)
        self.cap.set(cv2.CAP_PROP_GAIN, 255)
        self.cap.set(cv2.CAP_PROP_SHARPNESS, 150)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 2000)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)

        self.height = 26 
        self.camera_angle = 33
        self.data = json.load(open('locations.json'))
        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), self.cap.get(cv2.CAP_PROP_FPS))

        if not self.cap.isOpened():
            raise IOError("Could not open webcam!")

        options = apriltag.DetectorOptions(
            families='tag16h5',
            border=10,
            nthreads=6,
            quad_decimate=1.0,
            quad_blur=3.0,
            refine_edges=True,
            refine_decode=True,
            refine_pose=True,
            debug=False,
            quad_contours=True)
        tag_size = 6

        self.axis_const = [
                            [-tag_size / 2, -tag_size / 2, 0], 
                            [-tag_size / 2, tag_size / 2, 0], 
                            [tag_size / 2, tag_size / 2, 0], 
                            [tag_size / 2, -tag_size / 2, 0]
        ]

        self.detector = apriltag.Detector(options)

    def main(self):

        with np.load(rospy.get_param("~calibration_file")) as data:
            mtx = data['arr_0']
            dist = data['arr_1']
            newcameramtx = data['arr_2']
            roi = data['arr_3']

        while True:
            _, frame = self.cap.read()
            frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)

            x, y, w, h = roi
            frame = frame[y:y+h, x:x+w]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            results = self.detector.detect(gray)

            for r in results:
                # if r.pose_err > 1:
                #     continue

                if r.decision_margin < 2:
                    continue

                # print(r.decision_margin)
                
                print(r.corners)
                id = str(r.tag_id)
                _, rVec, tVec = cv2.solvePnP(self.get_axis(self.data[id]), np.mat(r.corners), mtx, dist)
                Rt = np.mat(cv2.Rodrigues(rVec)[0])
                R = Rt.transpose()
                pos = -R * tVec
                print(self.data[id]["Location"])
                print(pos)
                print(R)

    def get_axis(self, data):
        axis = []
        for [x,y,z] in self.axis_const:
            axis.append(
                [
                    x+data['X'],
                    y+data['Y'],
                    z+data['Z']-self.height,
                ]
            )
        return np.float32(axis)


if __name__ == '__main__':
    dvt = None
    try:
        dvt = VisionApriltags()
        dvt.main()
    except rospy.ROSInterruptException:
        pass
    dvt.cap.release()
    
