#!/usr/bin/env python

## Controls the airship altitude

import rospy
import time
import sys
from geometry_msgs.msg import TransformStamped
from airshippi_vicon.msg import Rotor
#from airshippi_vicon import testmodule

# Global params
P = 90
I = 0.5
D = 0
RATE_HZ = 5 # Hz
VICON_TOPIC = '/vicon/gal_airship/gal_airship'
#VICON_TOPIC = '/vicon/wand_test/wand_test'

class AltitudeController(object):
    def __init__(self, z_target=0):
	# Variables
        self.z_target = z_target
	self.z_current = 0
	self.pwm_out = 0
	self.direction_out = 0
	self.new_input = True
	self.integral = 0
	self.last_error = 0
	
	# Initialise node
        rospy.init_node('altitude_controller', anonymous=True)
	
	# Initialise publisher and subscriber
        self.pub = rospy.Publisher('altitude_rotor', Rotor, queue_size=1)
        rospy.Subscriber(VICON_TOPIC, TransformStamped, self.callback, queue_size=1)

    def callback(self, data):
        # Get vertical position
        self.z_current = data.transform.translation.z

	# Get error
	error = self.z_target - self.z_current

	# Get integral
	self.integral += error/RATE_HZ

	# Get derivative
	derivative = (error - self.last_error)*RATE_HZ

        # Get raw motor value
        raw_pwm = -(P*error + I*self.integral + D*derivative)

	# Save pwm and direction
	self.direction_out = int(raw_pwm > 0)
	raw_pwm = abs(raw_pwm)
	self.pwm_out = max(0, min(raw_pwm, 255))
	
	# Note the new input
	self.new_input = True

	# Update error
	self.last_error = error	

        # Log it
        #rospy.loginfo(rospy.get_caller_id() + '\nAltitude: %s, Target: %s, Thruster value: %s', z, self.z_target, self.pwm_out)

    def run(self):
	# Set rate
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
	    # Construct Rotor and publish it only if a new input was received in the time since the last publish
	    if self.new_input:
                rotor = Rotor(pwm=self.pwm_out, direction=self.direction_out)
		self.new_input = False
	    else:
		rotor = Rotor(pwm=0, direction=0)
            self.pub.publish(rotor)
	    rospy.loginfo("\nAltitude: %s, Target: %s, Thruster value: %s", self.z_current, self.z_target, rotor)
            rate.sleep()

def usage():
    return "%s z_target"%sys.argv[0]


if __name__ == '__main__':
    if len(sys.argv) == 2:
        z_target = float(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Setting desired altitude to %s"%(z_target))
    ac = AltitudeController(z_target)
    ac.run()
