#!/usr/bin/python

# Subscribes to /images_<> topic
# Modifies image via OpenCV
# Republishes to /image_<>_edit topic

# To run: rosrun autobot camera_edit.py

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import Int64
from cv_bridge import CvBridge, CvBridgeError

import cv2
import cv_utils

ENCODING = "bgr8"
DISPENSER_CYLINDER_SIZE = 45000

BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLACK = (0, 0, 0)

class CameraEdit():

    def __init__(self):

        ##------ Parameters ------##

        self.FRONT_COLOR = None
        self.FRONT_COLOR_PREVIOUS = None

        self.INTAKE_COLOR = None
        self.INTAKE_COLOR_PREVIOUS = None

        self.DISPENSER_COLOR = None
        self.DISPENSER_COLOR_PREVIOUS = None

        ##------ Subscribers ------##

        # front camera
        rospy.Subscriber('images_front', Image, self.camera_front_callback, queue_size=1)

        # intake camera
        rospy.Subscriber('images_intake', Image, self.camera_intake_callback, queue_size=1)

        # dispenser camera
        rospy.Subscriber('images_dispenser', Image, self.camera_dispenser_callback, queue_size=1)

        ##------ Publishers ------##

        # front camera
        self.pub_front_edit = rospy.Publisher('image_front_edit', Image, queue_size=10)
        self.pub_front_center = rospy.Publisher('front_cylinder_center', Pose2D, queue_size=10)  # for map marker viz + cylinder follower
        self.pub_front_size = rospy.Publisher('front_cylinder_size', Int64, queue_size=10)       # for cylinder following (auto_driver)
        self.pub_front_area = rospy.Publisher('front_cylinder_area', Int64, queue_size=10)
        self.pub_front_color = rospy.Publisher('front_cylinder_color', String, queue_size=10)    # for sorting pipeline

        # intake camera
        self.pub_intake_edit = rospy.Publisher('image_intake_edit', Image, queue_size=10)
        self.pub_intake_center = rospy.Publisher('intake_cylinder_center', Pose2D, queue_size=10)
        self.pub_intake_size = rospy.Publisher('intake_cylinder_size', Int64, queue_size=10)       #alisha
        self.pub_intake_area = rospy.Publisher('intake_cylinder_area', Int64, queue_size=10)
        self.pub_intake_color = rospy.Publisher('intake_cylinder_color', String, queue_size=10)

        # dispenser camera
        self.pub_dispenser_edit = rospy.Publisher('image_dispenser_edit', Image, queue_size=10)
        self.pub_dispenser_center = rospy.Publisher('dispenser_cylinder_center', Pose2D, queue_size=10) 
        self.pub_dispenser_color = rospy.Publisher('dispenser_cylinder_color', String, queue_size=10)   # for sorting pipeline
        self.pub_dispenser_area = rospy.Publisher('dispenser_cylinder_area', Int64, queue_size=10)


    ##--------- HELPER FUNCTIONS ---------##

    def ros_to_cv_image(self, bridge, msg):
        """Convert ROS image to CV2 image"""
        img = bridge.imgmsg_to_cv2(msg, desired_encoding=ENCODING)
        return img


    def cv_to_ros_image(self, bridge, img, pub):
        """Convert/Publish CV2 image to ROS image"""
        if img is not None:
            # convert CV2 image to ROS image
            ros_img = bridge.cv2_to_imgmsg(img, encoding=ENCODING)

            # publish ROS image
            pub.publish(ros_img)


    def find_color_center_area(self, IMG, CAMERA_TYPE, COLOR_ID, COLOR_ID_PREVIOUS, PUB_CENTER, PUB_AREA, PUB_COLOR):
        """Identify RED/GREEN cylinders in respective cameras.
        Draw bounding box + center on image.
        Publish edited image, center, color, and area.
        """
        # initialize variables
        center = None            # publish cylinder center point

        # find red/green cylinder
        mask_red, area_red, box_red, center_red = cv_utils.pick_cylinder(IMG, "red", CAMERA_TYPE)
        mask_green, area_green, box_green, center_green = cv_utils.pick_cylinder(IMG, "green", CAMERA_TYPE)

        # if red cylinder found (priority)
        if center_red:
            area, box, center = area_red, box_red, center_red
            color_found = RED
            COLOR_ID = "red"
        # if green cylinder found
        elif center_green:
            area, box, center = area_green, box_green, center_green
            color_found = GREEN
            COLOR_ID = "green"
        # nothing found
        else:
            COLOR_ID = None

        # draw box around cylinder
        if center is not None:
            # draw point at cylinder center
            cv2.circle(IMG, center, 3, color_found, 3)

            # draw box around cylinder
            cv2.drawContours(IMG, [box], 0, color_found, 3)

            # create pose2d message
            pose_msg = Pose2D()
            pose_msg.x = center[0]
            pose_msg.y = center[1]
            pose_msg.theta = 0

            # publish center pose of cylinder on image (for marker viz)
            PUB_CENTER.publish(pose_msg)
            PUB_AREA.publish(area)
            if CAMERA_TYPE == "front": self.pub_front_size.publish(cv_utils.area_of_box(box[0:3]))
            if CAMERA_TYPE == "intake": self.pub_intake_size.publish(cv_utils.area_of_box(box[0:3])) #alisha


        # if no cylinders detected
        else:
            # create pose2d message
            pose_msg = Pose2D()
            pose_msg.x = 0
            pose_msg.y = 0
            pose_msg.theta = 0

            # publish center pose of cylinder on image (for marker viz)
            PUB_CENTER.publish(pose_msg)
            PUB_AREA.publish(0)
            if CAMERA_TYPE == "front": self.pub_front_size.publish(0)

        # publish current color
        PUB_COLOR.publish(COLOR_ID)

        # print current color
        if (COLOR_ID_PREVIOUS != COLOR_ID):
            rospy.loginfo("{} camera: {}".format(CAMERA_TYPE, COLOR_ID))
        current = COLOR_ID

        # return modified image
        return IMG

    ##--------- CALLBACKS ---------##

    def camera_front_callback(self, msg):
        # Convert ROS image to CV2 image
        bridge = CvBridge()
        img = self.ros_to_cv_image(bridge, msg)

        ########################################
        # use imported opencv functions here

        img = self.find_color_center_area(  IMG                 = img, 
                                            CAMERA_TYPE         = "front", 
                                            COLOR_ID            = self.FRONT_COLOR,
                                            COLOR_ID_PREVIOUS   = self.FRONT_COLOR_PREVIOUS,
                                            PUB_CENTER          = self.pub_front_center,
                                            PUB_AREA            = self.pub_front_area,
                                            PUB_COLOR           = self.pub_front_color
                                            )

        ########################################

        # Convert/Publish  CV2 image to ROS image
        self.cv_to_ros_image(bridge, img, self.pub_front_edit)


    def camera_intake_callback(self, msg):
        # Convert ROS image to CV2 image
        bridge = CvBridge()
        img = self.ros_to_cv_image(bridge, msg)

        ########################################
        # use imported opencv functions here

        img = self.find_color_center_area(  IMG                 = img, 
                                            CAMERA_TYPE         = "intake", 
                                            COLOR_ID            = self.INTAKE_COLOR,
                                            COLOR_ID_PREVIOUS   = self.INTAKE_COLOR_PREVIOUS,
                                            PUB_CENTER          = self.pub_intake_center,
                                            PUB_AREA            = self.pub_intake_area,
                                            PUB_COLOR           = self.pub_intake_color
                                            )

        ########################################

        # Convert/Publish  CV2 image to ROS image
        self.cv_to_ros_image(bridge, img, self.pub_intake_edit)


    def camera_dispenser_callback(self, msg):
        # Convert ROS image to CV2 image
        bridge = CvBridge()
        img = self.ros_to_cv_image(bridge, msg)

        ########################################
        # use imported opencv functions here

        img = self.find_color_center_area(  IMG                 = img, 
                                            CAMERA_TYPE         = "dispenser", 
                                            COLOR_ID            = self.DISPENSER_COLOR,
                                            COLOR_ID_PREVIOUS   = self.DISPENSER_COLOR_PREVIOUS,
                                            PUB_CENTER          = self.pub_dispenser_center,
                                            PUB_AREA            = self.pub_dispenser_area,
                                            PUB_COLOR           = self.pub_dispenser_color
                                            )

        ########################################

        # Convert/Publish  CV2 image to ROS image
        self.cv_to_ros_image(bridge, img, self.pub_dispenser_edit)


if __name__ == "__main__":
    try:
        rospy.init_node('camera_edit_node')
        gp = CameraEdit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


