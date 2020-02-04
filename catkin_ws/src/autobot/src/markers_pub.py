#!/usr/bin/python

# To run: rosrun autobot markers_pub.py

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D

def callback(data):
    x, y, z, th = data.x, data.y, 0, data.theta
    print(x, y, z, th)
    b, g, r = 0, 0, 1
    viz_marker_point(x/10, y/10, z/10, b, g, r, "laser_frame", 1)

def viz_marker_point(x, y, z, b, g, r, frame_id, marker_id):
    rospy.loginfo("creating marker_point")
    marker = Marker()
    marker.type = marker.SPHERE
    marker.action = marker.ADD

    #marker.ns = ns
    marker.header.frame_id = frame_id
    marker.id = marker_id

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.b = b
    marker.color.g = g
    marker.color.r = r
    marker.color.a = 1.0
    pub_marker_point.publish(marker)

if __name__=="__main__":
    try:
        # initialize ROS node
        rospy.init_node('viz_markers')

        # initialize cylinder points subscribers
        rospy.Subscriber('front_cylinder_center', Pose2D, callback)

        # initialize marker publishers
        pub_marker_point = rospy.Publisher('markers', Marker, queue_size=10)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

