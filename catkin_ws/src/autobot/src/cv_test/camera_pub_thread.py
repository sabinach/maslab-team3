#!/usr/bin/python

# Publishes rgb images from multiple logitech cameras
# To run: rosrun autobot camera_pub.py

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import threading

FLIP = False

FRONT = True
DISPENSER = True

VIDEO_PORT_FRONT = 0
VIDEO_PORT_DISPENSER = 2


class camThread(threading.Thread):
    def __init__(self, camera_name, video_port, pub):
        threading.Thread.__init__(self)
        self.camera_name = camera_name
        self.video_port = video_port
        self.pub = pub
    def run(self):
        print("Starting " + self.camera_name)
        camPreview(self.video_port, self.pub)



def camPreview(previewName, camID):
    cv2.namedWindow(previewName)
    cam = cv2.VideoCapture(camID)
    if cam.isOpened():
        rval, frame = cam.read()
    else:
        rval = False

    while rval:
        cv2.imshow(previewName, frame)
        rval, frame = cam.read()
        key = cv2.waitKey(20)
        if key == 27:  # exit on ESC
            break
    cv2.destroyWindow(previewName)



def camPreview(video_port, pub):
    cap = cv2.VideoCapture(video_port)

    # capture frame-by-frame
    if cap.isOpened():
        rval, frame = cap.read()
    else:
        rval = False

    while rval:
        rval, frame = cap.read()

        if FLIP:
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

    # free resources
    cap.release()


if __name__=="__main__":
    try:
        # initialize ROS node
        rospy.init_node('camera_node')

        # publisher
        pub_img_front = rospy.Publisher('images', Image, queue_size=1)
        if DISPENSER: pub_img_dispenser = rospy.Publisher('images_dispenser', Image, queue_size=1)

        # Create threads as follows
        if FRONT:
            thread1 = camThread("Camera Front", VIDEO_PORT_FRONT, pub_img_front)
            thread1.start()
        if DISPENSER:
            thread2 = camThread("Camera Dispenser", VIDEO_PORT_DISPENSER, pub_img_dispenser)
            thread2.start()
        print("Active threads", threading.activeCount())

    except rospy.ROSInterruptException:
        pass