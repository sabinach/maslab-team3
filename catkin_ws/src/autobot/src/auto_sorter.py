#!/usr/bin/env python

# Autonomously sort the cylinders
# 1. Intake motor (always running)
# 2. Sense cylinder color on bottom
# 3. Move dispenser motor to correct side while still exploring
# 4. Completely move dispenser motor to correct side after confirming cylinder intake
# 4. Sense cylinder exists on dispenser
# 5. Push cylinder to correct side via dispenser motor
# 6. Turn servo to drop cylinder
# ....repeat steps 1-6 to gather more cylinders
# ....move to goal                                                      (NO LONGER ATTEMPTING)
# 7. Push cylinder out of the holder onto the pegs via outtake motor    (NO LONGER ATTEMPTING)

# To run: rosrun autobot auto_sorter.py

import rospy
import kitware.msg
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Int64

import sys
sys.path.insert(0, "/home/team3/team-3/catkin_ws/src/autobot/src/gamepad")
import gamepad_mappings

import time

# Rate to check for keypresses and send commands
RATE = 10

# toggle electronics
INTAKE_MOTOR = True
DISPENSER_MOTOR = True
DISPENSER_SERVO = True

# movement mappings
STOP = (0, 0)           # (left wheel, right wheel)
INTAKE_MOVE = (-1, 0)
DISPENSER_LEFT = (-1, 0)
DISPENSER_RIGHT = (1, 0)

# cap max movement speeds
INTAKE_MAX_SPEED = 1.5
DISPENSER_MOTOR_MAX_SPEED = 0.3
DISPENSER_SERVO_SPEED = 120

# sorter states
INTAKE_MODE = 0                 # looking for cylinders, change dispenser motor position to match incoming cylinder
DISPENSER_FINALIZE_MODE = 1     # finalize dispenser position to prep for dispenser detection
DISPENSER_DETECT_MODE = 2       # cylinder detected in dispenser
DISPENSER_SORT_MODE = 3         # push to correct side
DISPENSER_SERVO_TURN_MODE = 4   # turn servo
DISPENSER_SERVO_RESET_MODE = 5  # reset servo

# state dictionary
state = {INTAKE_MODE:               "INTAKE: Exploring; adjusting dispenser",
         DISPENSER_FINALIZE_MODE:   "DIS-FINALIZE: Detected cylinder in intake; finalizing dispenser position",
         DISPENSER_DETECT_MODE:     "DIS-DETECT: Waiting for dispenser camera to detect cylinder",
         DISPENSER_SORT_MODE:       "DIS-SORT: Detected cylinder in dispenser; sorting.. ",
         DISPENSER_SERVO_TURN_MODE: "DIS-SERVO-TURN: Finalized sort; turning servo",
         DISPENSER_SERVO_RESET_MODE:"DIS-SERVO-RESET: Reset servo"
         }

# tuned parameters
INTAKE_CYLINDER_CONFIRMED_AREA = 50000       # robot about to intake cylinder, for sure got it!
DISPENSER_CYLINDER_CONFIRMED_AREA = 5000     # robot confirmed that cylinder exists on cylinder
SERVO_MAX = 70

# max turn is irrelevant if DISPENSER_SERVO_SPEED > SERVO_MAX
SERVO_MAX = DISPENSER_SERVO_SPEED if DISPENSER_SERVO_SPEED > SERVO_MAX else SERVO_MAX


# dispenser motor encoder values
DISPENSER_MOTOR_ERROR_MARGIN = 1000
GREEN_RESET_VALUE = 0
RED_RESET_VALUE = 15000
GREEN_SORTED_VALUE = 11000
RED_SORTED_VALUE = 9000

# sanity checking
DEBUG = False


class AutoSorter:

    def __init__(self):

        ### ----------- Subscribers ----------- ###

        # Subscriber for joy commands
        rospy.Subscriber('joy', Joy, self.joy_callback)

        # front camera
        rospy.Subscriber('front_cylinder_color', String, self.front_cylinder_color_callback, queue_size=1)
        rospy.Subscriber('front_cylinder_area', Int64, self.front_cylinder_area_callback, queue_size=1)

        # intake camera
        rospy.Subscriber('intake_cylinder_color', String, self.intake_cylinder_color_callback, queue_size=1)
        rospy.Subscriber('intake_cylinder_area', Int64, self.intake_cylinder_area_callback, queue_size=1)

        # dispenser camera
        rospy.Subscriber('dispenser_cylinder_color', String, self.dispenser_cylinder_color_callback, queue_size=1)
        rospy.Subscriber('dispenser_cylinder_area', Int64, self.dispenser_cylinder_area_callback, queue_size=1)

        # dispenser servo values
        rospy.Subscriber('dispenser_servo_val', Int64, self.dispenser_servo_val_callback, queue_size=1)

        ### ----------- Subscribers (for encoders) ----------- ###

        # dispenser motor encoder
        rospy.Subscriber('dispenser_motor_encoder_val', Int64, self.dispenser_motor_encoder_val_callback, queue_size=10)


        ### ----------- Publishers ----------- ###

        # Publisher for intake motor commands
        self.pub_intake_cmd = rospy.Publisher('intake_cmd', kitware.msg.DriveCMD, queue_size=10)

        # Publisher for dispenser motor commands
        self.pub_dispenser_cmd = rospy.Publisher('dispenser_cmd', kitware.msg.DriveCMD, queue_size=10)

        # Publisher for speed commands
        self.pub_dispenser_servo_speed = rospy.Publisher('dispenser_servo_speed', Int64, queue_size=10)


        ### ----------- Variables ----------- ###

        # callback variables
        self.front_cylinder_color = None
        self.front_cylinder_area = None
        self.intake_cylinder_color = None
        self.intake_cylinder_area = None
        self.dispenser_cylinder_color = None
        self.dispenser_cylinder_area = None
        self.dispenser_servo_val = None
        self.dispenser_motor_encoder_val = None

        # initial states
        self.STATE = INTAKE_MODE
        self.STATE_PREVIOUS = None

        # initial triggers
        self.SORTING_COLOR = None
        self.DISPENSER_FINALIZED = False
        self.DISPENSER_SORTED = False
        self.DISPENSER_RESET = False
        self.SERVO_TURNED = False
        self.SERVO_RESET = False

        ### ----------- General ----------- ###

        # button down button
        self.AUTO_SORT_TOGGLE = False

        # set publisher rate
        rate = rospy.Rate(RATE)

        # main body loop
        while not rospy.is_shutdown():
            #self.auto_sort()                             # NOT using joystick
            if self.AUTO_SORT_TOGGLE: self.auto_sort()    # using joystick
            rate.sleep()

    ### -------------------- Callbacks -------------------- ###

    def joy_callback(self, msg):
        """ Set joystick mappings from gamepad callback.
        Start sorting process if button_down pressed."""
        mappings = gamepad_mappings.set_gamepad_mappings(msg)
        if mappings["button_rb"] == 1:
            if DEBUG: print("Autosort Button DOWN")
            self.AUTO_SORT_TOGGLE = True
        else:
            if DEBUG: print("Autosort Button UP")
            self.AUTO_SORT_TOGGLE = False
            self.stop_all_drive_cmds()

    def front_cylinder_color_callback(self, msg):
        """Get COLOR of detected cylinder of FRONT camera."""
        self.front_cylinder_color = msg.data

    def front_cylinder_area_callback(self, msg):
        """Get AREA of detected cylinder from FRONT camera"""
        self.front_cylinder_area = msg.data

    def intake_cylinder_color_callback(self, msg):
        """Get COLOR of detected cylinder of INTAKE camera."""
        self.intake_cylinder_color = msg.data

    def intake_cylinder_area_callback(self, msg):
        """Get AREA of detected cylinder from INTAKE camera"""
        self.intake_cylinder_area = msg.data

    def dispenser_cylinder_color_callback(self, msg):
        """Get COLOR of detected cylinder of DISPENSER camera."""
        self.dispenser_cylinder_color = msg.data

    def dispenser_cylinder_area_callback(self, msg):
        """Get AREA of detected cylinder from DISPENSER camera"""
        self.dispenser_cylinder_area = msg.data

        # If at any point dispenser recognizes a color in the dispenser, it should attempt to sort + use servo
        if self.STATE != DISPENSER_SERVO_TURN_MODE:
            if self.dispenser_cylinder_area > DISPENSER_CYLINDER_CONFIRMED_AREA:    # cylinder has been detected in dispenser!
                rospy.loginfo("{} detected in DISPENSER".format(self.dispenser_cylinder_color))
                self.STATE = DISPENSER_SERVO_TURN_MODE                                    # change state!

    def dispenser_servo_val_callback(self, msg):
        """Get updated dispenser servo value."""
        self.dispenser_servo_val = msg.data

    def dispenser_motor_encoder_val_callback(self, msg):
        """Get updated dispenser motor encoder value."""
        self.dispenser_motor_encoder_val = msg.data


    ### -------------------- Electronics -------------------- ###

    def publish_drive_cmd(self, pub, drive_speed):
        # publish intake motor speed
        drive_msg = kitware.msg.DriveCMD()
        drive_msg.l_speed, drive_msg.r_speed = drive_speed
        pub.publish(drive_msg)


    def stop_all_drive_cmds(self):
        self.publish_drive_cmd(self.pub_intake_cmd, STOP)
        self.publish_drive_cmd(self.pub_dispenser_cmd, STOP)
        self.pub_dispenser_servo_speed.publish(0)


    def start_intake_motor(self):
        """Run intake motor continuously."""
        # set intake motor speed
        left_motor = INTAKE_MOVE[0] * INTAKE_MAX_SPEED
        right_motor = INTAKE_MOVE[1] * INTAKE_MAX_SPEED
        intake_speed = (left_motor, right_motor)
        # publish drive cmd
        self.publish_drive_cmd(self.pub_intake_cmd, intake_speed)


    def prepare_dispenser_motor(self):
        """Move dispenser RIGHT/LEFT if GREEN/RED cylinder detected in front camera"""
        # initialize to 0
        left_motor, right_motor = STOP

        # green: reset
        if self.front_cylinder_color == "green":
            if DEBUG: print("green")
            if self.dispenser_motor_encoder_val > GREEN_RESET_VALUE + DISPENSER_MOTOR_ERROR_MARGIN:
                if DEBUG: print("go right")
                left_motor = DISPENSER_RIGHT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_RIGHT[1] * DISPENSER_MOTOR_MAX_SPEED

        # red: reset
        elif self.front_cylinder_color == "red":
            if DEBUG: print("red")
            if self.dispenser_motor_encoder_val < RED_RESET_VALUE - DISPENSER_MOTOR_ERROR_MARGIN:
                if DEBUG: print("go left")
                left_motor = DISPENSER_LEFT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_LEFT[1] * DISPENSER_MOTOR_MAX_SPEED

        else:
            if DEBUG: print("NONE")
            left_motor, right_motor = STOP

        # set dispenser speed commands
        dispenser_speed = (left_motor, right_motor)

        # publish drive cmd
        self.publish_drive_cmd(self.pub_dispenser_cmd, dispenser_speed)


    def finalize_dispenser_motor(self):
        """Finish moving dispenser RIGHT/LEFT after cylinder to intake is confirmed"""
        # initialize to 0
        left_motor, right_motor = STOP

        # green: reset
        if self.SORTING_COLOR == "green":
            if DEBUG: print("green FINAL")
            if self.dispenser_motor_encoder_val > GREEN_RESET_VALUE + DISPENSER_MOTOR_ERROR_MARGIN:
                if DEBUG: print("go right")
                left_motor = DISPENSER_RIGHT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_RIGHT[1] * DISPENSER_MOTOR_MAX_SPEED
            else:
                self.DISPENSER_FINALIZED = True

        # red: reset
        elif self.SORTING_COLOR == "red":
            if DEBUG: print("red FINAL")
            if self.dispenser_motor_encoder_val < RED_RESET_VALUE - DISPENSER_MOTOR_ERROR_MARGIN:
                if DEBUG: print("go left")
                left_motor = DISPENSER_LEFT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_LEFT[1] * DISPENSER_MOTOR_MAX_SPEED
            else:
                self.DISPENSER_FINALIZED = True

        else:
            if DEBUG: print("NONE")
            left_motor, right_motor = STOP

        # set dispenser speed commands
        dispenser_speed = (left_motor, right_motor)

        # publish drive cmd
        self.publish_drive_cmd(self.pub_dispenser_cmd, dispenser_speed)


    def sort_dispenser_motor(self):
        """Push cylinder in dispenser to correct sorting location (basically opposite of reset)"""
        # initialize to 0
        left_motor, right_motor = STOP

        # green: reset
        if self.SORTING_COLOR == "green":

            # within range
            if abs(self.dispenser_motor_encoder_val - GREEN_SORTED_VALUE) < DISPENSER_MOTOR_ERROR_MARGIN:
                self.DISPENSER_SORTED = True
                left_motor, right_motor = STOP

            # too small, increase value
            elif self.dispenser_motor_encoder_val < GREEN_SORTED_VALUE - DISPENSER_MOTOR_ERROR_MARGIN:
                left_motor = DISPENSER_LEFT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_LEFT[1] * DISPENSER_MOTOR_MAX_SPEED

            # too large, decrease value
            elif self.dispenser_motor_encoder_val > GREEN_SORTED_VALUE + DISPENSER_MOTOR_ERROR_MARGIN:
                left_motor = DISPENSER_RIGHT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_RIGHT[1] * DISPENSER_MOTOR_MAX_SPEED

        # red: reset
        elif self.SORTING_COLOR == "red":
            # within range
            if abs(self.dispenser_motor_encoder_val - RED_SORTED_VALUE) < DISPENSER_MOTOR_ERROR_MARGIN:
                self.DISPENSER_SORTED = True
                left_motor, right_motor = STOP

            # too small, increase value
            elif self.dispenser_motor_encoder_val < RED_SORTED_VALUE - DISPENSER_MOTOR_ERROR_MARGIN:
                left_motor = DISPENSER_LEFT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_LEFT[1] * DISPENSER_MOTOR_MAX_SPEED

            # too large, decrease value
            elif self.dispenser_motor_encoder_val > RED_SORTED_VALUE + DISPENSER_MOTOR_ERROR_MARGIN:
                left_motor = DISPENSER_RIGHT[0] * DISPENSER_MOTOR_MAX_SPEED
                right_motor = DISPENSER_RIGHT[1] * DISPENSER_MOTOR_MAX_SPEED

        else:
            left_motor, right_motor = STOP

        # set dispenser speed commands
        dispenser_speed = (left_motor, right_motor)

        # publish drive cmd
        self.publish_drive_cmd(self.pub_dispenser_cmd, dispenser_speed)


    def turn_dispenser_servo(self):
        """Turn servo if cylinder detected in dispenser"""
        print('turn -- go to {} degrees'.format(SERVO_MAX))
        if self.dispenser_servo_val != 0:
            self.dispenser_servo_speed = DISPENSER_SERVO_SPEED
            print('HELLOOOO')
            self.pub_dispenser_servo_speed.publish(self.dispenser_servo_speed)
        else:
            self.SERVO_TURNED = True

    def reset_dispenser_servo(self):
        """Reset servo position"""
        if self.dispenser_servo_val > 0:
            self.dispenser_servo_speed = -DISPENSER_SERVO_SPEED
            self.pub_dispenser_servo_speed.publish(self.dispenser_servo_speed)
        else:
            self.SERVO_RESET = True


    ### -------------------- Autonomous -------------------- ###

    # front: looks for cylinders
    # intakes: 100% sure in intake
    # dispenser: 100% sure in dispenser
    def auto_sort(self):
        # always on
        if INTAKE_MOTOR: self.start_intake_motor()

	'''
        # STATE 0: intake
        # looking for cylinders, change dispenser motor position to match incoming cylinder
        if self.STATE == INTAKE_MODE:
            # reset states
            self.SORTING_COLOR = None
            self.DISPENSER_FINALIZED = False
            self.DISPENSER_SORTED = False
            self.DISPENSER_RESET = False
            self.SERVO_TURNED = False
            self.SERVO_RESET = False
            # start electronics
            if DISPENSER_MOTOR: self.prepare_dispenser_motor()              # reset dispenser motor to prepare for intake (right: red, left: green)
            if self.intake_cylinder_area > INTAKE_CYLINDER_CONFIRMED_AREA:  # confirmed that robot will intake this cylinder
                rospy.loginfo("{} confirmed in INTAKE".format(self.intake_cylinder_color.upper()))
                self.SORTING_COLOR = self.intake_cylinder_color             # set sorting color for this sorting pipeline
                self.STATE = DISPENSER_FINALIZE_MODE                        # change state!

        # STATE 1: Finalize dispenser
        # go to final dispenser preparation position completely
        elif self.STATE == DISPENSER_FINALIZE_MODE:
            if DISPENSER_MOTOR: self.finalize_dispenser_motor()             # go to final preparation position completely
            if self.DISPENSER_FINALIZED:
                rospy.loginfo("DISPENSER finalized to receive {} cylinder".format(self.SORTING_COLOR.upper()))
                self.STATE = DISPENSER_DETECT_MODE                          # change state!

        # STATE 2: Wait to detect cylinder in dispenser
        # cylinder detected in dispenser
        elif self.STATE == DISPENSER_DETECT_MODE:
            if self.dispenser_cylinder_area > DISPENSER_CYLINDER_CONFIRMED_AREA:                # cylinder has been detected in dispenser!
                rospy.loginfo("{} detected in DISPENSER".format(self.dispenser_cylinder_color.upper()))
                self.STATE = DISPENSER_SERVO_TURN_MODE                                                # change state!


        # STATE 4: Turn servo
        elif self.STATE == DISPENSER_SERVO_TURN_MODE:
            if DISPENSER_SERVO: self.turn_dispenser_servo()         # turn servo to drop cylinder into container
            if self.SERVO_TURNED:
                rospy.loginfo("Servo TURNED")
                self.STATE = DISPENSER_SERVO_RESET_MODE             # change state!
            time.sleep(3)

        # STATE 5: Reset servo
        elif self.STATE == DISPENSER_SERVO_RESET_MODE:
            if DISPENSER_SERVO: self.reset_dispenser_servo()        # reset servo position
            if self.SERVO_RESET:                                    # servo is done, and has reset
                rospy.loginfo("Servo RESET")
                self.STATE = INTAKE_MODE                            # change state: go back to intake!

	'''

        # print current state
        if (self.STATE_PREVIOUS != self.STATE):
            rospy.loginfo("STATE ({}): {}".format(self.STATE, state[self.STATE]))
        self.STATE_PREVIOUS = self.STATE


if __name__=="__main__":
    try:
        rospy.init_node("auto_sorter")
        aus = AutoSorter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    '''
    # STATE 3: Push dispenser motor to appropriate side
    # push cylinder to correct side of dispenser
    elif self.STATE == DISPENSER_SORT_MODE:
        if DISPENSER_MOTOR: self.sort_dispenser_motor()  # push cylinder to correct dispenser location
        if self.DISPENSER_SORTED:  # successfully sorted
            rospy.loginfo("{} cylinder SORTED".format(self.dispenser_cylinder_color.upper()))
            self.STATE = DISPENSER_SERVO_TURN_MODE  # change state!
    '''
