#!/usr/bin/env python

import rospy
import kitware.msg
from tamproxy import ROSSketch
from tamproxy.devices import AnalogInput, Motor


class KitBot(ROSSketch):
    """ROS Node that controls the KitBot via the Teensy and tamproxy"""

    # current mappings
    # left motor: M1, right motor: M2
    # green: GND
    # blue: 2, pwm2
    # purple: 3, dir2
    # grey: 4, pwm1
    # white: 5, dir1

    # PIN MAPPINGS
    LMOTOR_PINS = (5, 4)  # DIR1 (white), PWM1 (grey)
    RMOTOR_PINS = (3, 2)  # DIR2 (purple), PWM2 (blue)

    def setup(self):
        """One-time method that sets up the robot, like in Arduino"""
        # Create the motor objects
        self.lmotor = Motor(self.tamp, *self.LMOTOR_PINS)
        self.rmotor = Motor(self.tamp, *self.RMOTOR_PINS)

        # Create a subscriber to listen for drive motor commands
        self.drive_sub = rospy.Subscriber('drive_cmd', kitware.msg.DriveCMD, self.drive_callback)

    def loop(self):
        """Method that loops at a fast rate, like in Arduino"""
        pass

    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return speed > 0, int(abs(speed * 255))

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.lmotor.write(*self.speed_to_dir_pwm(msg.l_speed))
        self.rmotor.write(*self.speed_to_dir_pwm(msg.r_speed))


if __name__ == '__main__':
    kb = KitBot(rate=100)  # Run at 100Hz (10ms loop)
    kb.run()
