#!/usr/bin/env python

# Turns servo to 180 degrees when button is pressed
# To run: rosrun autobot dispenser_servo.py

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64

import sys
sys.path.insert(0, "/home/team3/team-3/catkin_ws/src/autobot/src/gamepad")
import gamepad_mappings

# servo speeds
SERVO_DISPENSER_SPEED = 120

# servo max turn
SERVO_MAX = 10

# max turn is irrelevant if DISPENSER_SERVO_SPEED > SERVO_MAX
SERVO_MAX = SERVO_DISPENSER_SPEED if SERVO_DISPENSER_SPEED > SERVO_MAX else SERVO_MAX


class DispenserServo:

    def __init__(self):
        # Subscriber for joy commands
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Subscriber for dispenser servo values
        rospy.Subscriber('/dispenser_servo_val', Int64, self.servo_dispenser_val_callback)

        # Publisher for speed commands
        self.pub_servo_dispenser_speed = rospy.Publisher('dispenser_servo_speed', Int64, queue_size=10)

        # initial servo dispenser parameters
        self.servo_dispenser_val = None
        self.servo_dispenser_speed = None

        # initial gamepad parameters
        self.move_vertical = None
        self.move_horizontal = None


    def joy_callback(self, msg):
        """ Set joystick mappings from gamepad callback."""
        mappings = gamepad_mappings.set_gamepad_mappings(msg)
        self.move_vertical = mappings["button_vertical"]        # up: +1.0, down: -1.0
        self.move_horizontal = mappings["button_horizontal"]    # left: +1.0, right: -1.0


    def servo_dispenser_val_callback(self, msg):
        """Update servo dispenser value"""

        # callback
        self.servo_dispenser_val = msg.data

        # don't move if multiple buttons pressed, or none are pressed
        if (self.move_horizontal == 0 and self.move_vertical == 0):
            print('reset -- return to 0 degrees')
            if self.servo_dispenser_val != 0:
                self.servo_dispenser_speed = -SERVO_DISPENSER_SPEED
                self.pub_servo_dispenser_speed.publish(self.servo_dispenser_speed)

        # up (+)
        elif (self.move_vertical > 0):
            print('turn -- go to {} degrees'.format(SERVO_MAX))
            if self.servo_dispenser_val < SERVO_MAX:
                self.servo_dispenser_speed = SERVO_DISPENSER_SPEED
                self.pub_servo_dispenser_speed.publish(self.servo_dispenser_speed)



if __name__=="__main__":
    try:
        rospy.init_node("electronics")
        sd = DispenserServo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass