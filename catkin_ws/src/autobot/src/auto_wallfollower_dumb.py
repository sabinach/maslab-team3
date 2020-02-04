#!/usr/bin/env python

# Follow wall autonomously
# To run: rosrun autobot auto_wallfollower.py

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
import kitware.msg
from random import randrange
import numpy as np
import math

# Rate to loop through rospy main body
RATE = 10
DISTANCE = 0.3

# speed mappings
LEFT = (-1, 1)    # (left wheel, right wheel)
RIGHT = (1, -1)
FORWARD = (1, 1)
BACKWARD = (-1, -1)
STOP = (0, 0)

# cap min/max movement speeds
MAX_SPEED = 0.4
MIN_SPEED = 0.25

# Parameters
N = 100        # running avg window
RIGHT = ()   # (min_angle, max_angle)

# debugging
DEBUG = False



class WallFollower:

    def __init__(self):

        # Subscribers
        rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.pub_auto_drive = rospy.Publisher('auto_drive', kitware.msg.DriveCMD, queue_size=10)


        # Publishers
        self.pub_scan_clean = rospy.Publisher('scan_clean', LaserScan, queue_size=10)

        # initialize marker publishers
        self.pub_marker_point = rospy.Publisher('markers', Marker, queue_size=10)

        # set publisher rate
        rate = rospy.Rate(RATE)

        self.drive_speed = STOP

        self.turning = []

        # main body loop
        while not rospy.is_shutdown():
            if not self.turning:
                self.follow_wall()
            else:
                self.drive_speed = self.turning.pop(0)
            self.set_drive_cmds()
            rate.sleep()


    def scan_callback(self, msg):
        if DEBUG: self.stack_angle_range(msg)

        # scan metadata
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.ranges = msg.ranges

        # clean the range data (average out 0.0 values)
        self.cleaned_ranges = self.clean_ranges(self.ranges)

        # stack angle and ranges
        self.stacked_AR = self.stack_angle_range(self.angle_min, self.angle_max, self.angle_increment, self.cleaned_ranges)

        # crop the range of data used for wall following
        self.cropped_AR = self.crop_data(self.stacked_AR)

        # get coordinates from angles/ranges
        self.coords = self.get_coords(self.cropped_AR)

        # get linear regression line
        self.linreg = self.get_lineregression(self.self.coords)


    def stack_angle_range(self, angle_min, angle_max, angle_increment, ranges):
        angles = np.array([np.arange(angle_min, angle_max-angle_increment, angle_increment)]).T
        ranges = np.array([ranges]).T
        stacked = np.hstack((angles, ranges))
        return stacked


    def clean_ranges(self, ranges):
        # get running average
        convolved = np.convolve(ranges, np.ones((N,)) / N, mode='same')

        # get only the average of ranges that were 0
        multiplied = (ranges==0) * convolved

        # re-add back the averaged-out 0 ranges into the original ranges list
        filtered_ranges = np.add(ranges, multiplied)

        # numpy list
        return filtered_ranges


    def crop_data(self, data):
        # only use positive half
        cropped_data = data[data.shape[0] // 2:, :]
        return cropped_data


    def get_coords(self, stacked_data):
        coords = []
        counter = 1
        for angle, range in stacked_data:
            x, y = range*math.cos(angle), range*math.sin(angle)
            b, g, r = (255, 0, 0)
            coords.append((x,y))
            self.viz_marker_point(x, y, 0, b, g, r, "laser_frame", counter)
            counter += 1
        return coords

    def random_wandering(self):
        """
        This function defines the linear.x and angular.z velocities for the random wandering of the robot.
        Returns:
                Twist(): msg with angular and linear velocities to be published
                        msg.linear.x -> [0.1, 0.3]
                        msg.angular.z -> [-1, 1]
        """
        TURN = randrange(10)%2

        if not TURN:
            front = self.cleaned_ranges[-30:30]
            front_distance = sum(front)/60
            if front_distance > DISTANCE:
                left_motor = MAX_SPEED * FORWARD[0]
                right_motor = MAX_SPEED * FORWARD[1]
        else:
            left_motor = MAX_SPEED*RIGHT[0]
            right_motor = MAX_SPEED*RIGHT[1]
            self.turning = [(left_motor,right_motor)]*randrange(4)

        self.drive_speed = (left_motor, right_motor)

    def follow_wall(self):
        front = self.cleaned_ranges[-30:30]
        left = self.cleaned_ranges[510:570]
        front_distance = sum(front)/60
        left_distance = sum(left)/60
        left_motor = 0
        right_motor = 0
        if left_distance < DISTANCE and front_distance < DISTANCE:
            left_motor = MAX_SPEED*RIGHT[0]
            right_motor = MAX_SPEED*RIGHT[1]
            self.turning = [(left_motor, right_motor)]*3
        elif left_distance < DISTANCE:
            if front_distance > DISTANCE*2:
                left_motor = MAX_SPEED * FORWARD[0]
                right_motor = MAX_SPEED * FORWARD[1]
            else:
                left_motor = MAX_SPEED * FORWARD[0] * 0.75
                right_motor = MAX_SPEED * FORWARD[1] * 0.75
        elif front_distance < DISTANCE:
            left_motor = MAX_SPEED*LEFT[0]
            right_motor = MAX_SPEED*LEFT[1]
            self.turning = [(left_motor, right_motor)]*3
        else:
            self.random_wandering()

        self.drive_speed = (left_motor, right_motor)

    def auto_drive(self):
        """Publish drive commands to autobot autonomously"""
        auto_drive_msg = kitware.msg.DriveCMD()
        auto_drive_msg.l_speed, auto_drive_msg.r_speed = self.drive_speed
        self.pub_auto_drive.publish(auto_drive_msg)

if __name__=="__main__":
    try:
        rospy.init_node("auto_wallfollower_node")
        gp = WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    '''
    if DEBUG:
        # pub ranges (for debugging)
        scan_msg = LaserScan()
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = msg.time_increment
        scan_msg.scan_time = msg.scan_time
        scan_msg.range_min = msg.range_min
        scan_msg.range_max = msg.range_max
        scan_msg.ranges = self.ranges_clean
        scan_msg.intensities = msg.intensities
        self.pub_scan_clean.publish(scan_msg)
    '''
