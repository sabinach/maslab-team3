#!/usr/bin/env python

# Autonomously look for and drive into cylinders (for intake), ignoring walls (pls)
# 1: Explore -- look for cylinders by wall following (right wall)
# 2: Toward cylinder -- if cylinder detected, drive towards it and ram into it (ignore goals / dispensers / AR tags pls)
# 3: Intake cylinder -- ram into cylinder? move/don't move during intake/sorting process?

# To run: rosrun autobot auto_driver.py

import rospy
import kitware.msg
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int64, Float32

import numpy as np

# Rate to check for keypresses and send commands
RATE = 10

# speed mappings
STOP = (0, 0)
LEFT = (-1, 1)
RIGHT = (1, -1)

# cap min/max movement speeds
MAX_SPEED = 0.4
MIN_SPEED = 0.25

# camera dimensions
FRONT_DIM = (640, 480)  # w, h

# competition states
EXPLORE = 0               # look for cylinder
TO_CYLINDER_FRONT = 1     # move towards cylinder
TO_CYLINDER_INTAKE = 2    # continue moving if cylinder found

state  = {EXPLORE: "EXPLORE",
          TO_CYLINDER_FRONT: "To cylinder FRONT",
          TO_CYLINDER_INTAKE: "To cylinder INTAKE"}

# competition timer
MAX_TIME = 250  # seconds

# tuning parameters
CYLINDER_DISTANCE_FACTOR = 1        # TODO: what does this do again?
CYLINDER_THRESHOLD_AREA = 4000      # contour area threshold (when to start slowing down)

# wall following
GOAL_DIST = 0.3


class Autonomous:

    def __init__(self):

        # Initial states
        self.STATE = TO_CYLINDER_FRONT
        self.STATE_PREVIOUS = None
        self.drive_speed = STOP

        # Variables
        self.direction_front = 0    # -1 to 1: -1 means turn left, 0 forward, 1 turn right
        self.direction_intake = 0   # -1 to 1: -1 means turn left, 0 forward, 1 turn right
        self.speed_front = 0        # 0 to 2
        self.speed_intake = 0       # 0 to 2
        self.current_dist = 0       # wall following


        # Subscribers
        rospy.Subscriber('front_cylinder_center', Pose2D, self.front_cylinder_center_callback, queue_size=1)
        rospy.Subscriber('front_cylinder_size', Int64, self.front_cylinder_size_callback, queue_size=1)

        rospy.Subscriber('intake_cylinder_center', Pose2D, self.intake_cylinder_center_callback, queue_size=1)
        rospy.Subscriber('intake_cylinder_size', Int64, self.intake_cylinder_size_callback, queue_size=1)

        #rospy.Subscriber('perp_distance', Float32, self.perp_distance_callback, queue_size=1)

        # Publishers
        self.pub_auto_drive = rospy.Publisher('auto_drive', kitware.msg.DriveCMD, queue_size=10)

        # set publisher rate
        rate = rospy.Rate(RATE)

        # main body loop
        while not rospy.is_shutdown():
            self.auto_drive()
            self.set_drive_cmds()
            rate.sleep()

    ### -------------------- Callbacks -------------------- ###

    def front_cylinder_center_callback(self, msg):
        """Calculate detected cylinder center's error offset from image center.
        Set turn direction (-1: left, +1: right) based on error ratio."""

        # no cylinder detected
        if (msg.x==-1 and msg.y==-1) or (msg.x==0 and msg.y==0):
            self.direction_front = None

        # cylinder detected
        else:
            w, h = FRONT_DIM
            w_center, h_center = (w // 2, h // 2)
            error = msg.x - w_center
            self.direction_front = error / w_center

    def front_cylinder_size_callback(self, msg):
        """Calculate drive speed depending on how far semibot is from the cylinder."""

        # no cylinder detected
        if msg.data==-1 or msg.data==0:
            self.speed_front = None

        # cylinder detected
        else:
            if msg.data < CYLINDER_THRESHOLD_AREA: self.speed_front = 2.         # cylinder far -> full speed
            else: self.speed_front = 2. * CYLINDER_THRESHOLD_AREA / msg.data     # cylinder close -> slow down


    def intake_cylinder_center_callback(self, msg):
        # no cylinder detected
        if (msg.x==-1 and msg.y==-1) or (msg.x==0 and msg.y==0):
            self.direction_intake = None
        # cylinder detected
        else:
            w, h = FRONT_DIM
            w_center, h_center = (w // 2, h // 2)
            error = msg.x - w_center
            self.direction_intake = error / w_center

    def intake_cylinder_size_callback(self, msg):
        """Calculate drive speed depending on how far semibot is from the cylinder."""
        # no cylinder detected
        if msg.data==-1 or msg.data==0:
            self.speed_intake = None

        # cylinder detected
        else:
            if msg.data < CYLINDER_THRESHOLD_AREA: self.speed_intake = 2.         # cylinder far -> full speed
            else: self.speed_intake = 2. * CYLINDER_THRESHOLD_AREA / msg.data     # cylinder close -> slow down


    ### -------------------- Follow Cylinder -------------------- ###


    def to_cylinder_mode(self, direction, speed):
        """Set drive_speed based on cylinder center location + cylinder size.
        Turn left/right based on cylinder center location wrt to image center.
        Move fast/slow based on cylinder contour area size."""

        # no cylinder detected
        if (direction is None) or (speed is None):
            self.drive_speed = STOP

        # cylinder detected
        else:
            # TODO: what does this do again?
            left_motor = min((speed + direction) / 2., speed / 2.) * CYLINDER_DISTANCE_FACTOR
            right_motor = min((speed - direction) / 2., speed / 2.) * CYLINDER_DISTANCE_FACTOR

            # cap max speed
            left_motor = left_motor if abs(left_motor) < MAX_SPEED else np.sign(left_motor) * MAX_SPEED
            right_motor = right_motor if abs(right_motor) < MAX_SPEED else np.sign(right_motor) * MAX_SPEED

            # cap minimum speed
            left_motor = left_motor if abs(left_motor) > MIN_SPEED else np.sign(left_motor) * MIN_SPEED
            right_motor = right_motor if abs(right_motor) > MIN_SPEED else np.sign(right_motor) * MIN_SPEED

            # set drive_speed
            self.drive_speed = (left_motor, right_motor)

    def find_cylinder(self):
        # randomly start turning LEFT
        left_motor = LEFT[0] * MIN_SPEED
        right_motor = LEFT[1] * MIN_SPEED
        self.drive_speed = (left_motor, right_motor)

    def publish_drive_cmd(self, pub, drive_speed):
        # publish intake motor speed
        drive_msg = kitware.msg.DriveCMD()
        drive_msg.l_speed, drive_msg.r_speed = drive_speed
        pub.publish(drive_msg)


    def start_intake_motor(self):
        """Run intake motor continuously."""
        # set intake motor speed
        left_motor = INTAKE_MOVE[0] * INTAKE_MAX_SPEED
        right_motor = INTAKE_MOVE[1] * INTAKE_MAX_SPEED
        intake_speed = (left_motor, right_motor)
        # publish drive cmd
        self.publish_drive_cmd(self.pub_intake_cmd, intake_speed)


    ### -------------------- Autonomous -------------------- ###


    def auto_drive(self):
        """Publish drive commands to autobot autonomously"""
        auto_drive_msg = kitware.msg.DriveCMD()
        auto_drive_msg.l_speed, auto_drive_msg.r_speed = self.drive_speed
        self.pub_auto_drive.publish(auto_drive_msg)


    def set_drive_cmds(self):
        """Publish drive speed to kitware from joystick commands.
        """

        # set state based on which camera detects
        if (self.direction_front is None) and (self.direction_intake is None):
            self.STATE = EXPLORE
        elif self.direction_intake is not None:
            self.STATE = TO_CYLINDER_INTAKE
        elif self.direction_front is not None:
            self.STATE = TO_CYLINDER_FRONT


        # move differently depending on state
        if self.STATE == EXPLORE:
            self.find_cylinder()

        elif self.STATE == TO_CYLINDER_FRONT:
            self.to_cylinder_mode(self.direction_front, self.speed_front)

        elif self.STATE == TO_CYLINDER_INTAKE:
            self.to_cylinder_mode(self.direction_intake, self.speed_intake)


        # print current state
        if (self.STATE_PREVIOUS != self.STATE):
            rospy.loginfo("STATE ({}): {}".format(self.STATE, state[self.STATE]))
        self.STATE_PREVIOUS = self.STATE


if __name__=="__main__":
    try:
        rospy.init_node("auto_driver")
        gp = Autonomous()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        '''
        if self.STATE == FOLLOW_WALL:
            self.follow_wall()
        
        def perp_distance_callback(self, msg):
            # current distance
            self.current_dist = msg.data

        def follow_wall(self):
            """Follow the RIGHT wall"""

            # don't move initially
            left_motor, right_motor = STOP

            constant_speed = 0.3

            kp = 0.4
            error = self.current_dist - GOAL_DIST
            p = kp * error

            # too close, turn left
            if error > 0:
                left_motor = constant_speed - p
                right_motor = constant_speed + p

            # too far, turn right
            elif error < 0:
                left_motor = constant_speed + p
                right_motor = constant_speed - p

            # set drive_speed
            self.drive_speed = (left_motor, right_motor)

        '''
