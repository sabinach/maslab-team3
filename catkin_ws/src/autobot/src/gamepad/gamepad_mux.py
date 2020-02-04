#!/usr/bin/env python

# Integrate deadman switches for manual, autonomous, and competition mode
# LB: manual, RB: auto, START: competition, BACK: shutdown
# Adapted from: https://github.com/fishberg/racecar_mn/blob/master/src/mux.py

import rospy
import kitware.msg
from sensor_msgs.msg import Joy

import gamepad_mappings

# drive_speed movement mappings
STOP = (0, 0)
MUX_MODE = None

# debugging
PREVIOUS_MUX_MODE = None


##-------- Gamepad -------- ##


def joy_callback(msg):
    global MUX_MODE
    global PREVIOUS_MUX_MODE

    # get gamepad mappings
    mapping = gamepad_mappings.set_gamepad_mappings(msg)
    manual = mapping["button_lb"]
    auto = mapping["button_rb"]
    competition = mapping["button_start"]
    shutdown = mapping["button_back"]

    # TODO START: TESTING DIGITAL ELECTRONICS
    # intake motor
    intake_motor = mapping["button_horizontal"]     # left button

    # dispenser servo
    dispenser_servo = mapping["button_vertical"]    # up button

    # dispenser motor
    dispenser_left = mapping["button_x"]
    dispenser_right = mapping["button_b"]

    # auto sorter
    auto_sorter = mapping["button_vertical"]        # down button


    # TODO END: TESTING DIGITAL ELECTRONICS

    # if LB is pressed, enable teleop
    if manual == 1:
        MUX_MODE = "manual"

    # if RB is pressed, enable autonomy
    elif auto == 1:
        MUX_MODE = "auto"

    # if START button is pressed, enable 250 sec competition mode
    elif competition == 1:
        MUX_MODE = "competition"

    # if END button is pressed, HARD shutdown system
    elif shutdown == 1:
        MUX_MODE = "shutdown"

    # TODO START: TESTING DIGITAL ELECTRONICS

    elif intake_motor == 1:
        MUX_MODE = "intake_motor"

    elif dispenser_servo == 1:
        MUX_MODE = "dispenser_servo"

    elif dispenser_left == 1:
        MUX_MODE = "dispenser_left"

    elif dispenser_right == 1:
        MUX_MODE = "dispenser_right"

    elif auto_sorter == -1:
        MUX_MODE = "auto_sorter"

    # TODO END: TESTING DIGITAL ELECTRONICS

    else:
        MUX_MODE = ""

    # print mode
    if (PREVIOUS_MUX_MODE != MUX_MODE) and (MUX_MODE != ""):
        rospy.loginfo(MUX_MODE)
    PREVIOUS_MUX_MODE = MUX_MODE


##-------- Drive Publishers -------- ##


def drive(drive_speed):
    """Publish drive commands to semibot
    """
    drive_cmd_msg = kitware.msg.DriveCMD()
    drive_cmd_msg.l_speed, drive_cmd_msg.r_speed = drive_speed
    pub_drive_cmd.publish(drive_cmd_msg)


##-------- Callbacks -------- ##


def gamepad_drive_callback(msg):
    global MUX_MODE
    if MUX_MODE == "manual":
        pub_drive_cmd.publish(msg)
    elif MUX_MODE == "":
        drive(STOP)


def auto_drive_callback(msg):
    global MUX_MODE
    if MUX_MODE == "auto":
        pub_drive_cmd.publish(msg)
    elif MUX_MODE == "":
        drive(STOP)


""" 
    elif MUX_MODE == "competition":
        pass

    elif MUX_MODE == "shutdown":
        pass

    elif MUX_MODE == "":
        drive(STOP)
"""


try:
    # initialize node
    rospy.init_node("gamepad_mux")

    ##-------- Publishers -------- ##

    # Publisher for drive commands
    pub_drive_cmd = rospy.Publisher('drive_cmd', kitware.msg.DriveCMD, queue_size=10)

    ##-------- Subscribers -------- ##

    # Subscriber to joystick and gamepad commands
    rospy.Subscriber('joy', Joy, joy_callback)

    # manual
    rospy.Subscriber('gamepad_drive', kitware.msg.DriveCMD, gamepad_drive_callback)

    # autonomous
    rospy.Subscriber('auto_drive', kitware.msg.DriveCMD, auto_drive_callback)

    # wait before shutdown
    rospy.spin()

except rospy.ROSInterruptException:
    pass
