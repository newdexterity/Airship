#!/usr/bin/env python

## Controls the airship yaw angle

import rospy
import time
import sys
from math import pi
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
from airshippi_vicon.msg import Rotor
from airshippi_vicon.msg import RotorPair

# Global params
# diagonal yaw:0.9
BASE_PWM = 20
P = 10
I = 0
D = 0
INVERT_LEFT = True
INVERT_RIGHT = False
RATE_HZ = 10 # Hz
VICON_TOPIC = '/vicon/gal_airship/gal_airship'
#VICON_TOPIC = '/vicon/wand_test/wand_test'

class YawController(object):
    def __init__(self, yaw_target=0):
	# Parameters
        self.yaw_target = yaw_target
	self.yaw_current = 0
	self.pwm_left = 0
	self.pwm_right = 0
	self.direction_left = 0
	self.direction_right = 0
	self.new_input = True
	self.integral = 0
	self.last_error = 0
	
	# Initialise node
        rospy.init_node('yaw_controller', anonymous=True)
	
	# Initialise publisher and subscriber
        self.pub = rospy.Publisher('yaw_rotors', RotorPair, queue_size=1)
        rospy.Subscriber(VICON_TOPIC, TransformStamped, self.callback, queue_size=1)

    def quaternion_to_euler(self, quaternion_vec):
	# Returns [r, p, y] in rad
	rot_q = quaternion_vec
        explicit_quat = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
        return euler_from_quaternion(explicit_quat)

    def callback(self, data):
        # Get yaw as the third element in the euler angle vector
        self.yaw_current = self.quaternion_to_euler(data.transform.rotation)[2]
	
	# Get error
	error = self.yaw_target - self.yaw_current

	# Get integral
	self.integral += error/RATE_HZ
	
	# Get derivative
	derivative = (error - self.last_error)*RATE_HZ

	# Get shift from the base pwm	
	pwm_shift = -(P*error + I*self.integral + D*derivative)

	# Get raw pwm
	left_pwm = BASE_PWM + pwm_shift
	right_pwm = BASE_PWM - pwm_shift

	# Save pwm and direction for left
	self.direction_left = int((left_pwm < 0) != INVERT_LEFT)
	left_pwm = abs(left_pwm)
	self.pwm_left = max(0, min(left_pwm, 255))

	# Save pwm and direction for right
	self.direction_right = int((right_pwm < 0) != INVERT_RIGHT)
	right_pwm = abs(right_pwm)
	self.pwm_right = max(0, min(right_pwm, 255))

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
                left = Rotor(pwm=self.pwm_left, direction=self.direction_left)
		right = Rotor(pwm=self.pwm_right, direction=self.direction_right)
		self.new_input = False
	    else:
		left = Rotor(pwm=0, direction=0)
		right = Rotor(pwm=0, direction=0)
            self.pub.publish(left, right)
	    rospy.loginfo("\nYaw: %s, Target: %s, Left: %s, Right: %s", self.yaw_current, self.yaw_target, left, right)
            rate.sleep()

def usage():
    return "%s yaw_target"%sys.argv[0]


if __name__ == '__main__':
    if len(sys.argv) == 2:
        yaw_target = float(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Setting desired yaw to %s"%(yaw_target))
    yc = YawController(yaw_target)
    yc.run()
