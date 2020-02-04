#!/usr/bin/env python

# Start intake motor; constantly running
# To run: rosrun autobot intake_motor.py

import rospy
import kitware.msg
from sensor_msgs.msg import Joy

import sys
sys.path.insert(0, "/home/team3/team-3/catkin_ws/src/autobot/src/gamepad")
import gamepad_mappings

# Rate to check for keypresses and send commands
RATE = 10

# drive_speed movement mappings
MOVE = (-1, 0)    # (left wheel, right wheel)
STOP = (0, 0)

# cap max movement speeds
MAX_SPEED = 0.5

class IntakeMotor:

    def __init__(self):
        # Subscriber for joy commands
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Publisher for drive commands
        self.pub_intake_cmd = rospy.Publisher('intake_cmd', kitware.msg.DriveCMD, queue_size=10)

        # don't move initially
        self.drive_speed = STOP

        # set publisher rate
        rate = rospy.Rate(RATE)

        # main body loop
        while not rospy.is_shutdown():
            self.intake_drive()
            rate.sleep()


    def intake_drive(self):
        """Publish drive commands to autobot autonomously"""
        intake_drive_msg = kitware.msg.DriveCMD()
        intake_drive_msg.l_speed, intake_drive_msg.r_speed = self.drive_speed
        self.pub_intake_cmd.publish(intake_drive_msg)


    def joy_callback(self, msg):
        """ Set joystick mappings from gamepad callback."""
        mappings = gamepad_mappings.set_gamepad_mappings(msg)
        self.set_intake_cmds(mappings)


    def set_intake_cmds(self, mappings):
        # get gamepad mapping for intake
        move_intake = mappings["button_horizontal"]     # left button

        # start motor for intake intake
        if move_intake == 1:
            left_motor = MOVE[0] * MAX_SPEED
            right_motor = MOVE[1] * MAX_SPEED

        # stop intake
        else:
            left_motor, right_motor = STOP

        # set drive speed
        self.drive_speed = (left_motor, right_motor)



if __name__=="__main__":
    try:
        rospy.init_node("intake_motor")
        gp = IntakeMotor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
