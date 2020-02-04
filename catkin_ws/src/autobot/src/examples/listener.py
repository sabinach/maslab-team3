#!/usr/bin/python

# Based on: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# To run: rosrun autobot listener.py

# ROS imports
import rospy
from std_msgs.msg import String


def callback(data):
    print("I heard {}".format(data.data))


if __name__=="__main__":
    try:
        # initialize ROS node
        rospy.init_node('listener_node')

        # initialize subscribers
        rospy.Subscriber('chatter', String, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



