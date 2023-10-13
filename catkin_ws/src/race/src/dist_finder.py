#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
	
	# angle range pairs generation: 
	#angles = [data.angle_min + (data.angle_increment) for i in range(len(data.ranges))]
	#angle_range_pairs = zip(angles, data.ranges)

	# Bound check the input angle
	angle = math.radians(angle)
	lidar_angle_bounds = (-math.pi/6, 210 * (math.pi / 180))
	if not (lidar_angle_bounds[0] < angle < lidar_angle_bounds[1]):
		rospy.loginfo("requested LIDAR range outside of bounds: (%d, %d)"%lidar_angle_bounds)	
		return -1.0

	# Calculate the corresponding index(s) in data.ranges to input angle
	# If it does not diivide evenly, then the range returned will be a combination of cieling and floor range
	# Calculate proportion of cieling / floor range
	idx = (1/data.angle_increment) * ( angle+(lidar_angle_bounds[0]) )
	idx_low = math.floor(idx)
	idx_high = math.ceil(idx)
	p = idx - idx_low

	# If the index into data.ranges divided evenly...
	if idx_low == idx_high:
		range = data.ranges(idx)

		# Any data.range that is outside of the LIDAR min/max range is unreliable
		if not (data.range_min < range < data.range_max) :
			rospy.loginfo("LIDAR scan at angle: %d is not within lidar range"%angle)
		return data.ranges(idx)
	# If the index into data.ranges did not divide evenly
	else:
		range1 = data.ranges[idx_low]
		range2 = data.ranges[idx_high]

		# Any range that is outside of the LIDAR min/max range is unreliable
		if not (data.range_min < range1 < data.range_max and data.range_min < range2 < data.range_max ):
			rospy.loginfo("LIDAR scan(s) at angle: %d are not within lidar range"%angle)
		return ((1-p)*range1) + (p*range2) 



def callback(data):
	global forward_projection

	theta = 50 # you need to try different values for theta
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	swing = math.radians(theta)

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	alpha = math.atan( (a * math.cos(swing) - b) /
						(a * math.sin(swing)) )
	AB = b * math.cos(alpha)
	CD = AB + (forward_projection * math.sin(alpha))
	error = desired_distance - CD

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_X/scan",LaserScan,callback)
	rospy.spin()
