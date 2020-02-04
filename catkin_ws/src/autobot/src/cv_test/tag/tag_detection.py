#!/usr/bin/python

# Use this file to find Apriltags and find the homography,
# Then figure out the distance and orientation of the robot w.r.t the tag

# To run: python2 aprilTag.py

import cv2
import numpy as np
from apriltags3 import Detector 
from math import sqrt, atan2, pi

cap = cv2.VideoCapture(3) # change this number to show the actual webcam video

def descale(img,n):
    height, width = img.shape[0:2]
    return cv2.resize(img,(int(width/n),int(height/n)))

def nothing(x):
    pass

cv2.namedWindow('frame',cv2.WINDOW_NORMAL)

while True:
    _, frame = cap.read()

    gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    # imagepath = './test/test_image_rotation_-40.png'
    # imagepath = './goalTag.jpg'
    # image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

    at_detector = Detector(searchpath=['apriltags/lib'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0
                       )

    result = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[808.86,807.91,297.96,244.21], tag_size=0.0975)
    if result != []:
      # print(result[0].pose_t)
      vec = (result[0].pose_R.T.dot(result[0].pose_t)).T[0]
      vec = vec/sqrt(vec.dot(vec))
      rad = 180*atan2(vec[0],vec[2])/pi
      print("vec", vec)
      print("angle between origin line and z of tag:", rad)

      vec1 = (result[0].pose_R.T.dot(np.array([[0],[0],[1]]))).T[0]
      vec1 = vec1/sqrt(vec1.dot(vec1))
      rad1 = 180*atan2(vec1[0],vec1[2])/pi
      print("vec1", vec1)
      print("angle between facing direction and z of tag:", rad1)

      dis = sqrt(result[0].pose_t.T[0].dot(result[0].pose_t.T[0]))
      print("distance is ", dis, "meters")

    cv2.imshow('frame',descale(frame,1))

    k = cv2.waitKey(100)
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
