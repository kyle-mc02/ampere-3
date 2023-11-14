#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
error = 0.0		# initialize the error
gap_threshold = 0.15

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

def find_gap(data):
	ranges = np.array(data)
	diffs = np.diff(ranges)
	diffs = np.absolute(diffs)
	deepest = 0
	deepest_ind = -1
	# end_ind = -1
	for i in range(1,len(diffs)):
		if math.isnan(ranges[i]):
			# extended_ranges[i] = 0 #ranges[i-1]
			continue
		# if diffs[i-1] == deepest and diffs[i] < diffs[i-1]:
		#	 end_ind = i-1
		if (ranges[i] > deepest) and (not math.isinf(ranges[i])) and (not math.isnan(ranges[i])): # and data[i] < 4:
			deepest = ranges[i]
			deepest_ind = i
	return (deepest_ind)# + end_ind) //2

def callback(data):
	ranges = data.ranges
	# ranges = np.array(ranges[90:-90])
	ranges = np.array(ranges[90:-90])

	middle_index = len(ranges) // 2
	# gap_ind = find_gap(ranges)
	gap_ind = np.argmax(np.array(ranges))

	# print("{}: data.ranges".format(data.ranges))
	# print("{}: ranges len".format(len(data.ranges)))
	
	gap_offset = gap_ind - middle_index
	# print("{}: gap_offset".format(gap_offset))
	# this is not working currently
	# want to vary the error depending on how far away the object at the middle index is in comparison to the deepest gap
	# need to change how it is implemented
	# print('distance_ahead: {}'.format(ahead_distance))
	error = gap_offset * data.angle_increment 
	# error = ((gap_offset) * data.angle_increment + (data.ranges[gap_ind]-data.ranges[middle_index])) / data.ranges[middle_index]
	# print("{}: error".format(error))
	# vel = 0 if data.ranges[middle_index] <= .35 else 1
	#if error > 1 or error < -1:
		#return  
	print('error: {}'.format(error))
	msg = pid_input()
	msg.pid_error = error # -error
	# msg.pid_vel = sum(data.ranges)/len(data.ranges)	# velocity error can also be sent.
	msg.pid_vel = 0
	pub.publish(msg)

if __name__ == '__main__':
	# print("Hokuyo LIDAR node started")
	rospy.init_node('follow_the_gap',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/extended_ranges",LaserScan,callback)
	rospy.spin()
