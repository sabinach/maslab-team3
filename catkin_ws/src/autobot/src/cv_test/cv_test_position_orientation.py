#!/usr/bin/python

# Use this file to extract the cylinder(s) and draw the bounding rectangular.
# Right now only the area and the four points of the rect are returned, 
# Extra heuristic to calculate the position and orientation of the cylinder could be added

# To run: python cv_test_position_orientation.py

import numpy as np
import cv2

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv_utils

cap = cv2.VideoCapture(0) # change this number to show the actual webcam video
color_to_pick = "red"

def averageAdd(newimg,addList,n):
    if len(addList)<n:
        addList.append(newimg)
    else:
        addList = addList[1:]+[newimg]
    average = addList[0]
    for i in addList:
        average = cv2.bitwise_or(average,i)
    return average,addList

addList = []
while True:
    _, frame = cap.read()
    img_mask, mask = cv_utils.filter_color(frame, color_to_pick)
    # do an average add to stabalize, decrease the number if it's laggy
    average_mask, addList = averageAdd(mask,addList,5)

    kernel = np.ones((5, 5), np.float32) # kernel for erode and dilate
    # here we first erode then dilate to get rid of the small noise dots
    eroded = cv2.erode(average_mask,kernel)
    dilated = cv2.dilate(eroded, kernel)
    # cv2.imshow('average_mask',average_mask)
    # cv2.imshow('dilated',dilated)

    _, contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    threshold = 1000 # threshold for large contour

    # TO DO: the following code needs to be modified if we want to count multiple cylinders.
    cylinder_contour = None
    maxArea = threshold
    for cnt in contours:
        if cv2.contourArea(cnt) > maxArea:
        	maxArea = cv2.contourArea(cnt)
        	cylinder_contour = cnt
    if cylinder_contour is None:
    	continue
    else:
    	rect = cv2.minAreaRect(cylinder_contour)
    	print "area = ", cv2.contourArea(cylinder_contour)
    	box = cv2.boxPoints(rect)
    	box = np.int0(box)
    	print "points = ",box
    	img = cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)
    	cv2.imshow('cylinder',img)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
