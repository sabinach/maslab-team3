#!/usr/bin/env python

# All-encompassing ROSSketch for Semibot
# To run: rosrun autobot semibot.py

import rospy
from std_msgs.msg import Int64

import kitware.msg
from tamproxy import ROSSketch, Timer
from tamproxy.devices import AnalogInput, Motor, Servo, Encoder


##--------- PARAMETERS ---------##

RATE = 10                   # Rate to check for gamepad inputs and send commands
DISPENSOR_SPEED = 10        # Dispensor servo delta speed
ENCODER = True


##--------- PIN MAPPINGS ---------##

# Drive Motors
# left motor: M1, right motor: M2
# green: GND
# blue: 2, pwm2
# purple: 3, dir2
# grey: 4, pwm1
# white: 5, dir1
LMOTOR_PINS = (5, 4)  # DIR1 (white), PWM1 (grey)
RMOTOR_PINS = (3, 2)  # DIR2 (purple), PWM2 (blue)

# Intake Motor
# green: GND
# blue: 38, pwm
# purple: 37, dir
INTAKE_MOTOR_PINS = (22, 23)  # DIR (purple), PWM (blue)

# Dispenser Servo
SERVO_PIN = 10  # orange

# dispenser motor: M1 (left)
# green: GND
# blue: 6, pwm2
# purple: 7, dir2
# grey: 8, pwm1
# white: 9, dir1
DISPENSER_MOTOR_PINS = (9, 8)   # DIR1 (white), PWM1 (grey)
OUTTAKE_MOTOR_PINS = (7, 6)		# DIR2 (purple), PWM2 (blue)

# Encoders
LMOTOR_ENCODER_PINS = (14, 15)  # yellow, white
RMOTOR_ENCODER_PINS = (16, 17)
DISPENSER_MOTOR_ENCODER_PINS = (18, 19)


##--------- SEMIBOT ---------##


class SemiBot(ROSSketch):
    """ROS Node that controls the SemiBot via the Teensy and tamproxy"""

    def setup(self):
        """One-time method that sets up the robot, like in Arduino"""

        ##--------------------------------------------##
        ##--------- DRIVING MOTORS (+encoder) --------##
        ##--------------------------------------------##

        # Create the motor objects
        self.lmotor = Motor(self.tamp, *LMOTOR_PINS)
        self.rmotor = Motor(self.tamp, *RMOTOR_PINS)

        # Create a subscriber to listen for drive motor commands
        rospy.Subscriber('drive_cmd', kitware.msg.DriveCMD, self.drive_callback)

        ## --------- ##

        if ENCODER:
            # encoder objects
            self.lmotor_encoder = Encoder(self.tamp, *LMOTOR_ENCODER_PINS, continuous=True)
            self.rmotor_encoder = Encoder(self.tamp, *RMOTOR_ENCODER_PINS, continuous=True)

            # Publishers for encoder values
            self.pub_lmotor_encoder_val = rospy.Publisher('lmotor_encoder_val', Int64, queue_size=10)
            self.pub_rmotor_encoder_val = rospy.Publisher('rmotor_encoder_val', Int64, queue_size=10)


        ##--------------------------------------------##
        ##--------------- INTAKE MOTOR ---------------##
        ##--------------------------------------------##

        # Create the motor object
        self.intake_motor = Motor(self.tamp, *INTAKE_MOTOR_PINS)

        # Create a subscriber to listen for intake motor commands (passively continuous)
        rospy.Subscriber('intake_cmd', kitware.msg.DriveCMD, self.intake_callback)


        ##--------------------------------------------##
        ##-------------- DISPENSER SERVO -------------##
        ##--------------------------------------------##

        # Setup servo
        self.servo = Servo(self.tamp, SERVO_PIN)
        self.servo.write(0)
        self.servo_val = 0
        self.delta = DISPENSOR_SPEED
        self.end = False

        # Subscriber to listen for drive motor commands
        rospy.Subscriber('dispenser_servo_speed', Int64, self.dispenser_servo_callback)

        # Publisher for current servo value
        self.pub_dispenser_servo_val = rospy.Publisher('dispenser_servo_val', Int64, queue_size=10)


        ##--------------------------------------------##
        ##--------- DISPENSER MOTOR (+encoder) -------##
        ##--------------------------------------------##

        # Create the motor object
        self.dispenser_motor = Motor(self.tamp, *DISPENSER_MOTOR_PINS)

        # Create a subscriber to listen for dispenser motor commands
        rospy.Subscriber('dispenser_cmd', kitware.msg.DriveCMD, self.dispenser_motor_callback)

        ## --------- ##

        if ENCODER:
            # encoder objects
            self.dispenser_motor_encoder = Encoder(self.tamp, *DISPENSER_MOTOR_ENCODER_PINS, continuous=True)

            # Publishers for encoder values
            self.pub_dispenser_motor_encoder_val = rospy.Publisher('dispenser_motor_encoder_val', Int64, queue_size=10)


        ##--------------------------------------------##
        ##---------- General ---------##
        ##--------------------------------------------##

        # timer for loop()
        self.timer = Timer()

        # reset all motor encoders
        self.lmotor_encoder.val = 0
        self.rmotor_encoder.val = 0
        self.dispenser_motor_encoder.val = 0

    ##--------- GENERAL ---------##

    def loop(self):
        """Method that loops at a fast rate, like in Arduino (isn't looping???)"""
        #rospy.loginfo("looping!")
        if self.timer.millis() > 100:
            self.timer.reset()
            self.publish_servo_val()
            self.publish_encoder_val()
        pass

    def speed_to_dir_pwm(self, speed):
        """Converts floating point speed (-1.0 to 1.0) to dir and pwm values"""
        speed = max(min(speed, 1), -1)
        return speed > 0, int(abs(speed * 255))

    ##--------- ENCODERS ---------##

    def publish_encoder_val(self):
        if ENCODER:
            self.pub_lmotor_encoder_val.publish(self.lmotor_encoder.val)
            self.pub_rmotor_encoder_val.publish(self.rmotor_encoder.val)
            self.pub_dispenser_motor_encoder_val.publish(self.dispenser_motor_encoder.val)


    ##--------- DRIVING MOTORS ---------##

    def drive_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.lmotor.write(*self.speed_to_dir_pwm(msg.l_speed))
        self.rmotor.write(*self.speed_to_dir_pwm(msg.r_speed))


    ##--------- INTAKE MOTOR ---------##

    def intake_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.intake_motor.write(*self.speed_to_dir_pwm(msg.l_speed))    # use lspeed


    ##--------- DISPENSER SERVO ---------##

    def publish_servo_val(self):
        self.pub_dispenser_servo_val.publish(self.servo_val)

    def dispenser_servo_callback(self, msg):
        # write to servo
        self.delta = msg.data
        self.servo_val += self.delta
        self.servo.write(abs(self.servo_val))


    ##--------- DISPENSER MOTOR ---------##

    def dispenser_motor_callback(self, msg):
        """Processes a new drive command and controls motors appropriately"""
        self.dispenser_motor.write(*self.speed_to_dir_pwm(msg.l_speed))  # use LEFT motor: l_speed


if __name__ == '__main__':
    sb = SemiBot(rate=100)  # Run at 100Hz (10ms loop)
    sb.run()
