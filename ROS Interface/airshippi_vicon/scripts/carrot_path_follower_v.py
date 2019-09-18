#!/usr/bin/env python

## Controls the airship yaw angle

import rospy
import time
import sys
from math import pi, atan2, sqrt
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
from airshippi_vicon.msg import Rotor
from airshippi_vicon.msg import RotorPair
from airshippi_vicon.Path2D import Path, Arc, Line

# ALIGN YAW WITH VICON X AXIS WHEN CREATING OBJECT

# Global params
INVERT_LEFT = True
INVERT_RIGHT = False
RATE_HZ = 10 # Hz
VICON_TOPIC = '/vicon/gal_airship/gal_airship'
#VICON_TOPIC = '/vicon/wand_test/wand_test'

# Carrot
BASE_PWM = 50
MIN_PWM = 0
P = 110		# Proportional controller gain
speed_exp = 1
I = 0.0
D = 0.0
VTP_LOOKAHEAD_IDX = 23	# How far ahead the VTP will be placed
PATH_DP = 0.1 		# Distance between path points [m]

# Define path
arc1 = Arc.Arc(start_point=[0, 2], centre=[0, 0], angle=360)
#line1 = Line.Line([0,0], [0,4])
path = Path.Path(segment_list=[arc1], dp=PATH_DP)
path.reposition(new_centre=[0, 0])
#path.plot()

class CarrotPathFollower(object):
    def __init__(self, path):
	# Parameters
        self.path = path
	self.yaw_target = 0
	self.yaw_current = 0
	self.pos_current = [0, 0, 0]
	self.vtp = [0, 0]
	self.pwm_left = 0
	self.pwm_right = 0
	self.direction_left = 0
	self.direction_right = 0
	self.new_input = True
	self.last_error = 0
	self.integral = 0
	self.time_prev = 0
	self.pos_prev = None
	self.speed = 0

	self.raw_error = 0
	self.error = 0
	
	# Initialise node
        rospy.init_node('carrot_path_follower', anonymous=True)
	
	# Initialise publisher and subscriber
        self.pub = rospy.Publisher('yaw_rotors', RotorPair, queue_size=1)
        rospy.Subscriber(VICON_TOPIC, TransformStamped, self.callback, queue_size=1)

    def quaternion_to_euler(self, quaternion_vec):
	# Returns [r, p, y] in rad
	rot_q = quaternion_vec
        explicit_quat = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
        return euler_from_quaternion(explicit_quat)

    def get_yaw_target(self, current_pos_x, current_pos_y):
	# Generate current_pos
	current_pos = [current_pos_x, current_pos_y]
	
	# Get closest point on path
	closest_idx = self.path.get_closest(current_pos)

	# Get VTP
	self.vtp = self.path.get_point(closest_idx + VTP_LOOKAHEAD_IDX)

	# Get angle between the Virtual Target Point and current position, wrt the x axis
	vtp_angle = atan2(self.vtp[1] - current_pos[1], self.vtp[0] - current_pos[0])
	
	return vtp_angle


    def callback(self, data):
	# Get current timestamp
	time_current = float(data.header.stamp.secs)

        # Get yaw as the third element in the euler angle vector
        self.yaw_current = self.quaternion_to_euler(data.transform.rotation)[2]

	# Get current position
	self.pos_current = data.transform.translation

	# Get speed 
	dt = time_current - self.time_prev
	if self.pos_prev is None:
	    self.pos_prev = self.pos_current
	    return
	if dt > 0.2:
	    dy = self.pos_current.y - self.pos_prev.y
	    dx = self.pos_current.x - self.pos_prev.x
	    self.speed = sqrt(dy**2 + dx**2) / dt
	    self.time_prev = time_current
	    self.pos_prev = self.pos_current
	
	# Get error
	self.yaw_target = self.get_yaw_target(self.pos_current.x, self.pos_current.y)
	error = self.yaw_target - self.yaw_current
	
	self.raw_error = error
	# Handle angle wrapping
	if error > pi:
	    error -= 2*pi
	elif error < -pi:
	    error += 2*pi
	self.error = error

	# Get integral
	self.integral += error/RATE_HZ
	
	# Get derivative
	derivative = (error - self.last_error)*RATE_HZ	
	
	# Get shift from the base pwm	
	pwm_shift = (P*error + I*self.integral + D*derivative)*self.speed**(speed_exp)

	# Get raw pwm
	left_pwm = BASE_PWM - pwm_shift
	right_pwm = BASE_PWM + pwm_shift

	# Save pwm and direction for left
	self.direction_left = int((left_pwm < 0) != INVERT_LEFT)
	left_pwm = abs(left_pwm)
	self.pwm_left = max(MIN_PWM, min(left_pwm, 255))

	# Save pwm and direction for right
	self.direction_right = int((right_pwm < 0) != INVERT_RIGHT)
	right_pwm = abs(right_pwm)
	self.pwm_right = max(MIN_PWM, min(right_pwm, 255))
	
	# Update error
	self.last_error = error		
	
	# Note the new input
	self.new_input = True

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
	    rospy.loginfo("\nCurrent_pos: %s, VTP: %s", self.pos_current, self.vtp)
	    rospy.loginfo("\nRaw error: %s, Modified error: %s", self.raw_error, self.error)
            rate.sleep()

def usage():
    return "%s yaw_target"%sys.argv[0]


if __name__ == '__main__':
    print("Starting carrot path follower.")
    cpf = CarrotPathFollower(path)
    cpf.run()
