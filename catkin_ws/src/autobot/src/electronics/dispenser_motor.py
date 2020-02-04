#!/usr/bin/env python

# Dispenser motor;
# Uses LEFT motor from motor controller (shares with pusher motor)
# To run: rosrun autobot dispenser_motor.py

import rospy
import kitware.msg
from sensor_msgs.msg import Joy

import sys
sys.path.insert(0, "/home/team3/team-3/catkin_ws/src/autobot/src/gamepad")
import gamepad_mappings

# Rate to check for keypresses and send commands
RATE = 10

# drive_speed movement mappings
LEFT = (-1, 0)    # (left motor, right motor)
RIGHT = (1, 0)
STOP = (0, 0)

# cap max movement speeds
MAX_SPEED = 0.5

class DispenserMotor:

    def __init__(self):
        # Subscriber for joy commands
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Publisher for drive commands
        self.pub_dispenser_cmd = rospy.Publisher('dispenser_cmd', kitware.msg.DriveCMD, queue_size=10)

        # Set intake motor speed
        self.drive_speed = STOP

        # set publisher rate
        rate = rospy.Rate(RATE)

        # main body loop
        while not rospy.is_shutdown():
            self.dispenser_drive()
            rate.sleep()


    def dispenser_drive(self):
        """Publish drive commands to dispenser motor"""
        dispenser_drive_msg = kitware.msg.DriveCMD()
        dispenser_drive_msg.l_speed, dispenser_drive_msg.r_speed = self.drive_speed
        self.pub_dispenser_cmd.publish(dispenser_drive_msg)


    def joy_callback(self, msg):
        """ Set joystick mappings from gamepad callback."""
        mappings = gamepad_mappings.set_gamepad_mappings(msg)
        self.set_dispenser_cmds(mappings)


    def set_dispenser_cmds(self, mappings):
        move_left = mappings["button_x"]
        move_right = mappings["button_b"]

        # if neither button pressed or BOTH buttons pressed at the same time
        if (move_left == 0 and move_right == 0) or (move_left == 1 and move_right == 1):
            left_motor, right_motor = STOP

        # move rack and pinion LEFT
        elif move_left == 1:
            left_motor = LEFT[0] * MAX_SPEED
            right_motor = LEFT[1] * MAX_SPEED

        # move rack and pinion RIGHT
        elif move_right == 1:
            left_motor = RIGHT[0] * MAX_SPEED
            right_motor = RIGHT[1] * MAX_SPEED

        # TODO: need to reset to 0, but not sure how at the moment

        # set drive speed
        self.drive_speed = (left_motor, right_motor)


if __name__=="__main__":
    try:
        rospy.init_node("dispenser_motor")
        gp = DispenserMotor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass