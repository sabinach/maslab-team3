#!/usr/bin/python

# Publishes rgb images from logitech camera
# To run: rosrun autobot camera_pub.py

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

FLIP_FRONT = False
FLIP_INTAKE = False
FLIP_DISPENSER = False

FRONT = True
INTAKE = True
DISPENSER = True

VIDEO_PORT_FRONT = 0 
VIDEO_PORT_INTAKE = 2
VIDEO_PORT_DISPENSER = 4

def publish_images(cap, pub, flip):

    # capture frame-by-frame
    _, frame = cap.read()

    # confirm frame is present
    if frame is not None:

        # flip image
        if flip:
            w, h = frame.shape[1], frame.shape[0]
            center = (w // 2, h // 2)
            angle = 180  # degrees
            scale = 1
            M = cv2.getRotationMatrix2D(center, angle, scale)
            frame = cv2.warpAffine(frame, M, (w, h))

        # convert CV2 image to ROS image
        bridge = CvBridge()
        encoding = "bgr8"
        ros_img = bridge.cv2_to_imgmsg(frame, encoding=encoding)

        # publish ROS image
        pub.publish(ros_img)

if __name__=="__main__":
    try:
        # initialize ROS node
        rospy.init_node('camera_node')

        # connect to camera and initialize publisher
        if FRONT:
            rospy.loginfo("starting FRONT camera")
            cap_front = cv2.VideoCapture(VIDEO_PORT_FRONT)
            pub_img_front = rospy.Publisher('images_front', Image, queue_size=1)
        if INTAKE:
            rospy.loginfo("starting INTAKE camera")
            cap_intake = cv2.VideoCapture(VIDEO_PORT_INTAKE)
            pub_img_intake = rospy.Publisher('images_intake', Image, queue_size=1)
        if DISPENSER:
            rospy.loginfo("starting DISPENSER camera")
            cap_dispenser = cv2.VideoCapture(VIDEO_PORT_DISPENSER)
            pub_img_dispenser = rospy.Publisher('images_dispenser', Image, queue_size=1)

        # main body loop
        while not rospy.is_shutdown():
            if FRONT:       publish_images(cap_front, pub_img_front, FLIP_FRONT)
            if INTAKE:      publish_images(cap_intake, pub_img_intake, FLIP_INTAKE)
            if DISPENSER:   publish_images(cap_dispenser, pub_img_dispenser, FLIP_DISPENSER)

        # free resources
        if FRONT:       cap_front.release()
        if INTAKE:      cap_intake.release()
        if DISPENSER:   cap_dispenser.release()

    except rospy.ROSInterruptException:
        pass

