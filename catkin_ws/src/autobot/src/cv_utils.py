import cv2
import numpy as np
from math import sqrt

def draw_rectangle(img):
	"""Draws blue rectangle on top left corner of image"""
	img = cv2.rectangle(img, (0,0), (100,100), (255,0,0), 3)
	return img

# filter out the desired color
def filter_color(img,color,camera):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	if camera == "front":
		if color == "red":
			minH = 0
			minS = 35
			minV = 0
			maxH = 18
			maxS = 255
			maxV = 255
		elif color == "green":
			minH = 45
			minS = 29
			minV = 71
			maxH = 90
			maxS = 218
			maxV = 255
	elif camera == "intake":
		if color == "red":
			minH = 0
			minS = 131
			minV = 137
			maxH = 28
			maxS = 255
			maxV = 255
		elif color == "green":
			minH = 35
			minS = 61
			minV = 27
			maxH = 88
			maxS = 200
			maxV = 255
	elif camera == "dispenser":
		if color == "red":
			minH = 0
			minS = 81
			minV = 39
			maxH = 7
			maxS = 255
			maxV = 255
		elif color == "green":
			minH = 52
			minS = 60
			minV = 0
			maxH = 95
			maxS = 255
			maxV = 147
	lower_th = np.array([minH,minS,minV])
	upper_th = np.array([maxH,maxS,maxV])
	mask = cv2.inRange(hsv,lower_th, upper_th)
	masked_img = cv2.bitwise_and(img, img, mask=mask)
	return masked_img, mask

# helper function
def averageAdd(newimg, addList, n):
	if len(addList) < n:
		addList.append(newimg)
	else:
		addList = addList[1:] + [newimg]
	average = addList[0]
	for i in addList:
		average = cv2.bitwise_or(average, i)
	return average, addList

# pick the cylinder of color
# return annotated image, rect area , four corner points, and center
def pick_cylinder(img,color,camera):
	img, mask = filter_color(img, color,camera)
	
	# do an average add to stabilize, decrease the number if it's laggy
	addList = []
	for i in range(5):
		average_mask, addList = averageAdd(mask, addList, 5)

	# here we first erode then dilate to get rid of the small noise dots
	kernel = np.ones((5, 5), np.float32) # kernel for erode and dilate
	eroded = cv2.erode(average_mask, kernel)
	dilated = cv2.dilate(eroded, kernel)

	if cv2.__version__[0] == '3':
		im2, contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	elif cv2.__version__[0] == '4':
		contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	threshold = 1000 # threshold for large contour
	
	# TO DO: the following code needs to be modified if we want to count multiple cylinders.
	cylinder_contour = None
	cylinder_center = None
	maxArea = threshold
	for cnt in contours:
		if cv2.contourArea(cnt) > maxArea:
			maxArea = cv2.contourArea(cnt)
			cylinder_contour = cnt

			# get contour center
			M = cv2.moments(cnt)
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
			cylinder_center = (cX, cY)

	if cylinder_contour is None:
		return img, None, None, None # no cylinder found
	else:
		rect = cv2.minAreaRect(cylinder_contour)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		return img, cv2.contourArea(cylinder_contour), box, cylinder_center

def area_of_box(points):
	assert(len(points)==3)
	dis_squares = []
	dis_squares.append((points[0][0]-points[1][0])**2+(points[0][1]-points[1][1])**2)
	dis_squares.append((points[0][0]-points[2][0])**2+(points[0][1]-points[2][1])**2)
	dis_squares.append((points[1][0]-points[2][0])**2+(points[1][1]-points[2][1])**2)
	dis_squares.sort()
	return int(sqrt(dis_squares[0]*dis_squares[1]))

