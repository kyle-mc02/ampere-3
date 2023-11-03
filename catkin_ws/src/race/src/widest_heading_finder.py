#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
gap_threshold = .2
vel = 15

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def find_widest_heading(data):
	ranges = np.array(data.ranges)
	disparities = np.diff(ranges)
	
	last_gap_index = 0
	max_width_gap_index = 0
	max_width_gap_length = 0
	for i in range(len(disparities)) :
		if abs(disparities[i]) > gap_threshold:
			if i-last_gap_index >= max_width_gap_length:
				max_width_gap_index = i
				max_width_gap_length = i-last_gap_index

			last_gap_index = i

	midpoint_index = ((2*max_width_gap_index) + max_width_gap_length ) / 2
	heading = (midpoint_index*data.angle_increment) - (math.pi/3) 
	return heading


def callback(data):
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.

	lidar_heading = find_widest_heading(data)
	lidar_range = data.range_max - data.range_min
	relative_heading = (lidar_heading - (math.pi/2))

	msg.pid_error = relative_heading
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('widest_heading_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("extended_ranges",LaserScan,callback)
	rospy.spin()
