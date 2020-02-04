#!/usr/bin/env python

# Follow wall autonomously
# To run: rosrun autobot auto_wallfollower.py

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

import numpy as np
import math
from scipy import stats

# Rate to loop through rospy main body
RATE = 10

# Parameters
N = 100             # running avg window
GOAL_DIST = 0.3

# color codes
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# debugging
VIZ = True

class WallFollower:

    def __init__(self):

        # Subscribers
        rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)

        # Publishers
        self.pub_scan_clean = rospy.Publisher('scan_clean', LaserScan, queue_size=10)
        self.pub_perp_distance = rospy.Publisher('perp_distance', Float32, queue_size=10)

        # initialize marker publishers
        self.pub_marker_point = rospy.Publisher('markers_raw', Marker, queue_size=10)
        self.pub_marker_linregress = rospy.Publisher('markers_linregress', Marker, queue_size=10)
        self.pub_marker_linperp = rospy.Publisher('markers_linperp', Marker, queue_size=10)

        # set publisher rate
        rate = rospy.Rate(RATE)

        # main body loop
        while not rospy.is_shutdown():
            pass
            rate.sleep()


    def scan_callback(self, msg):
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
        self.coords_raw = self.get_coords(self.cropped_AR)

        # get linear regression line
        slope, intercept = self.get_linear_regression(self.coords_raw)

        # get perpendicular distance from point to line
        perp_distance = self.get_perp_distance(slope, intercept)

        # publish perpendicular distance to wall
        self.pub_perp_distance.publish(perp_distance)



    def stack_angle_range(self, angle_min, angle_max, angle_increment, ranges):
        # np.array([angle1, range1], [angle2, range2]... [angleN, rangeN])
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
        cropped_data = data[data.shape[0] // 4:, :]
        return cropped_data


    def get_coords(self, stacked_data):
        # convert angles/ranges to xy coordinate frame
        coords = []
        counter = 0
        for angle, range in stacked_data:
            x, y = range*math.cos(angle), range*math.sin(angle)
            coords.append([x, y])
            if VIZ:
                counter += 1
                b, g, r = BLUE
                marker = self.viz_marker_point(x, y, 0, b, g, r, "laser_frame", counter)
                self.pub_marker_point.publish(marker)
        return np.array(coords)


    def get_linear_regression(self, coords):
        # get slope/intercept
        x_coord, y_coord = coords.T[0], coords.T[1]
        slope, intercept, r_value, p_value, std_err = stats.linregress(x_coord, y_coord)

        # visualize linregress in rviz
        if VIZ:
            b, g, r = GREEN
            x1, y1 = (-intercept/slope, 0)     # x-intercept
            x2, y2 = (0, intercept)            # y-intercept
            marker = self.viz_marker_line(x1, y1, x2, y2, b, g, r, "laser_frame", 0)
            self.pub_marker_linregress.publish(marker)

        return slope, intercept


    def get_perp_distance(self, slope, intercept):
        A, B, C = slope, -1, intercept
        x, y = 0, 0
        distance = abs(A*x + B*y + C)/math.sqrt(A**2 + B**2)
        return distance


    def viz_marker_point(self, x, y, z, b, g, r, frame_id, marker_id, scale=0.1):
        marker = Marker()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # marker.ns = ns
        marker.header.frame_id = frame_id
        marker.id = marker_id
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.b = b
        marker.color.g = g
        marker.color.r = r
        marker.color.a = 1.0
        return marker


    def viz_marker_line(self, x1, y1, x2, y2, b, g, r, frame_id, marker_id, scale=0.1):
        marker = Marker()
        marker.type = Marker.LINE_LIST
        marker.action = marker.ADD
        marker.header.frame_id = frame_id
        marker.id = marker_id
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.1
        segments = [(x1, y1, 0), (x2, y2, 0)]
        marker.points = [Point(x, y, z) for (x, y, z) in segments]
        return marker


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
