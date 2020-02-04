#!/usr/bin/python

# Based on: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# To run: rosrun autobot talker.py

# ROS imports
import rospy
from std_msgs.msg import String


def talker():
    # create message
    msg = "Hello World {}".format(rospy.get_time())
    print(msg)

    # publish message
    pub.publish(msg)

    # time delay
    rospy.Rate(10).sleep()


if __name__=="__main__":
    try:
        # initialize ROS node
        rospy.init_node('talker_node')

        # initialize publishers
        pub = rospy.Publisher('chatter', String, queue_size=10)

        # main body loop
        while not rospy.is_shutdown():
            talker()

    except rospy.ROSInterruptException:
        pass



