#!/usr/bin/env python

# Drive kitbot via joystick commands.
# Adapted from: https://github.mit.edu/maslab-2020/kitware/blob/master/scripts/kbd_driver.py

import rospy
import kitware.msg
from sensor_msgs.msg import Joy

import gamepad_mappings

# Rate to check for gamepad inputs and send commands
RATE = 10

# drive_speed movement mappings
LEFT = (-1, 1)    # (left wheel, right wheel)
RIGHT = (1, -1)
FORWARD = (1, 1)
BACKWARD = (-1, -1)
STOP = (0, 0)

# cap max movement speeds
MAX_SPEED = 0.3
MAX_TURN_SPEED = 0.2

class GamePad:

    def __init__(self):
        # Subscriber for joy commands
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Publisher for drive commands
        self.pub_gamepad_drive= rospy.Publisher('gamepad_drive', kitware.msg.DriveCMD, queue_size=10)

        # initial drive speed parameter
        self.drive_speed = STOP

        # set publisher rate
        rate = rospy.Rate(RATE)

        # main body loop
        while not rospy.is_shutdown():
            self.gamepad_drive()
            rate.sleep()


    def joy_callback(self, msg):
        """ Set joystick mappings from gamepad callback.
        """
        # Set drive_cmd based on current mapping values
        mappings = gamepad_mappings.set_gamepad_mappings(msg)
        self.set_drive_cmds(mappings)


    def gamepad_drive(self):
        """Publish drive commands to semibot via gamepad
        """
        gamepad_drive_msg = kitware.msg.DriveCMD()
        gamepad_drive_msg.l_speed, gamepad_drive_msg.r_speed = self.drive_speed
        self.pub_gamepad_drive.publish(gamepad_drive_msg)


    def set_drive_cmds(self, gamepad_mappings):
        """Publish drive speed to kitware from joystick commands.
        """
        # make drive speed variable
        move_vertical = gamepad_mappings["left_joy_vertical"]
        move_horizontal = gamepad_mappings["right_joy_horizontal"]

        # don't move if multiple buttons pressed, or none are pressed
        if (move_horizontal != 0 and move_vertical != 0) or (move_horizontal == 0 and move_vertical == 0):
            left_motor, right_motor = STOP
        # forwards (+)
        if move_vertical > 0:
            left_motor = FORWARD[0] * abs(move_vertical) * MAX_SPEED
            right_motor = FORWARD[1] * abs(move_vertical) * MAX_SPEED
        # backwards (-)
        elif move_vertical < 0:
            left_motor = BACKWARD[0] * abs(move_vertical) * MAX_SPEED
            right_motor = BACKWARD[1] * abs(move_vertical) * MAX_SPEED
        # left (+)
        elif move_horizontal > 0:
            left_motor = LEFT[0] * abs(move_horizontal) * MAX_TURN_SPEED
            right_motor = LEFT[1] * abs(move_horizontal) * MAX_TURN_SPEED
        # right (-)
        elif move_horizontal < 0:
            left_motor = RIGHT[0] * abs(move_horizontal) * MAX_TURN_SPEED
            right_motor = RIGHT[1] * abs(move_horizontal) * MAX_TURN_SPEED

        # set drive_speed
        self.drive_speed = (left_motor, right_motor)

if __name__=="__main__":
    try:
        rospy.init_node("gamepad_driver")
        gp = GamePad()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass