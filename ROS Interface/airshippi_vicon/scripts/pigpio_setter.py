#!/usr/bin/env python

## Reads the rotor pwm and direction values and sets them

import rospy
import pigpio
from airshippi_vicon.msg import Rotor
from airshippi_vicon.msg import RotorPair

VERT_EN = 10
VERT_PH = 9
LEFT_EN = 5
LEFT_PH = 6
RIGHT_EN = 13
RIGHT_PH = 19
HOR_MODE = 26
PWM_FREQ = 20000

class PigpioSetter(object):
    def __init__(self, z_target=0):

	# Initialise pigpio
	self.pi = initialise_pigpio()
	
	# Initialise node
        rospy.init_node('pigpio_setter', anonymous=True)
	
	# Initialise subscribers
        rospy.Subscriber('/altitude_rotor', Rotor, self.set_altitude_rotor, queue_size=1)
        rospy.Subscriber('/yaw_rotors', RotorPair, self.set_yaw_rotors, queue_size=1)

    def set_rotor(self, pwm, direction, enable_pin, phase_pin):
	# Set direction
	self.pi.write(phase_pin, direction)	
	# Set PWM
        self.pi.set_PWM_dutycycle(enable_pin, pwm)

    def set_altitude_rotor(self, rotor):
	self.set_rotor(rotor.pwm, rotor.direction, VERT_EN, VERT_PH)
	rospy.loginfo("\nSet vert rotor pwm: %s, direction: %s", rotor.pwm, rotor.direction)

    def set_yaw_rotors(self, rotors):
	# Set left rotor
	self.set_rotor(rotors.left.pwm, rotors.left.direction, LEFT_EN, LEFT_PH)
	# Set right rotor
	self.set_rotor(rotors.right.pwm, rotors.right.direction, RIGHT_EN, RIGHT_PH)
	rospy.loginfo("\nSet rotor pair pwms: %s, %s, directions: %s, %s", rotors.left.pwm, rotors.right.pwm, rotors.left.direction, rotors.right.direction)

    def run(self):
	rospy.spin()

def initialise_pigpio():
    # Get instance
    pi = pigpio.pi()
    
    # Set everything to 0
    for pin in (VERT_EN, VERT_PH, LEFT_EN, LEFT_PH, RIGHT_EN, RIGHT_PH):
        pi.write(pin, 0)

    # Set horizontal motors to EN/PH mode
    pi.write(HOR_MODE, 1)

    # Set PWM frequency
    pi.set_PWM_frequency(VERT_EN, PWM_FREQ)
    pi.set_PWM_frequency(LEFT_EN, PWM_FREQ)
    pi.set_PWM_frequency(RIGHT_EN, PWM_FREQ)

    return pi

if __name__ == '__main__':
    # Define shutdown hook
    rospy.on_shutdown(initialise_pigpio)
    # Start the Setter
    ps = PigpioSetter()
    ps.run()
