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
import rospkg

rospy.init_node('vision_apriltags', anonymous=True)


class VisionApriltags:
    def __init__(self):
        rospack = rospkg.RosPack()
        
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
        self.locations = json.load(open(rospack.get_path('vision_apriltags') + "/src/locations.json"))

        if not self.cap.isOpened():
            raise IOError("Could not open webcam!")
        
        # print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) + "x" + self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) + "@" + self.cap.get(cv2.CAP_PROP_FPS) + "FPS")

        options = apriltag.DetectorOptions(
            families='tag36h11',
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

        self.axis_const = np.mat([
                            [0, -tag_size / 2, -tag_size / 2,], 
                            [0, -tag_size / 2, tag_size / 2,], 
                            [0, tag_size / 2, tag_size / 2,], 
                            [0, tag_size / 2, -tag_size / 2,]
        ])

        self.detector = apriltag.Detector(options)
    
        
        with np.load(rospack.get_path('vision_apriltags') + "/src/" + rospy.get_param("~calibration_file")) as data:
            self.mtx = data['arr_0']
            self.dist = data['arr_1']
            self.newcameramtx = data['arr_2']
            self.roi = data['arr_3']

    def main(self):
        while not rospy.is_shutdown():
            _, frame = self.cap.read()
            frame = cv2.undistort(frame, self.mtx, self.dist, None, self.newcameramtx)

            # x, y, w, h = self.roi
            # frame = frame[y:y+h, x:x+w]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            results = self.detector.detect(gray)
            
            imgpts = []
            objpts = []
            tags = []

            for r in results:
                if r.decision_margin < 2:
                    continue
                
                imgpts.extend(r.corners)
                objpts.extend(self.get_points(r.tag_id).tolist())
                
                tags.append(r.tag_id)
                
                # print(R)
            
            if (len(tags) == 0):
                print("No tags detected!")
                continue
            
                
            print("Detecting tags", tags)
            
            _, rVec, tVec = cv2.solvePnP(np.mat(objpts), np.mat(imgpts), self.mtx, None)
            Rt = np.mat(cv2.Rodrigues(rVec)[0])
            R = Rt.transpose()
            pos = -R * tVec
            
            # print(R)
            print(rVec)
            # print(Rt)
            print(pos)
            
    def get_points(self, id):
        data = self.locations[str(id)]
                
        adjusted_pose = self.axis_const + np.array([data["X"], data["Z"], data["Y"]])
                
        return adjusted_pose
        # return self.axis_const
        


if __name__ == '__main__':
    dvt = None
    try:
        dvt = VisionApriltags()
        dvt.main()
    except rospy.ROSInterruptException:
        pass
    dvt.cap.release()
    
