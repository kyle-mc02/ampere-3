#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import copy
import threading

# Handle to the publisher that will publish on the extended_ranges topic. messsages of type 'LaserScan'
pub = rospy.Publisher('extended_ranges', LaserScan, queue_size=10)

gap_threshold = .15 # Threshold for determining if there is a disparity / gap between two ranges
car_half_width = .15 # The half width of the car for extending th esmall error 
extender_padding = 0.25 # How far beyond car_half_width the extender will project ranges. 

# callback_lock = threading.Lock()

def computeExtendedRanges(data):
	# print('len(data.ranges) original: {}'.format(len(data.ranges)))
	# data: single message from topic /scan
	# angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
	# Outputs length in meters to object with angle in lidar scan field of view
	# Make sure to take care of NaNs etc.

	# with callback_lock:
	ranges = np.array(data.ranges)
	extended_ranges = np.array(data.ranges)
	# print("data: {}".format(data.ranges))
	# print("ranges: {}".format(ranges))
	# ranges = [i for i in data.ranges]
	# extended_ranges = [i for i in data.ranges]

	for i in range(1, len(ranges)-1):
		# For dealing with nan values -- we may need to tweak this
		if math.isnan(ranges[i]):
			# ranges[i] = min(ranges[i-1], ranges[i+1])
			# extended_ranges[i] = ranges[i]
			if (not math.isnan(data.ranges[i+1])):
				ranges[i] = 10
				extended_ranges[i] = data.ranges[i+1]
			elif not math.isnan(data.ranges[i-1]):
				ranges[i] = 10
				extended_ranges[i] = data.ranges[i-1]

	# ct = 0
	# for i in ranges:
	# 	if math.isnan(i):
	# 		ct += 1
	# print("Number of nans in ranges: ", ct)
	# ct = 0
	# for i in extended_ranges:
	# 	if math.isnan(i):
	# 		ct += 1
	# print("Number of nans in extended_ranges: ", ct)

	disparities = np.diff(ranges)
	
	# ct = 0
	# ct_disp = 0
	# for i in disparities:
	# 	if math.isnan(i):
	# 		ct += 1
	# 	if abs(i) > gap_threshold:
	# 		ct_disp += 1
	# print("Number of nans in disparities: ", ct)
	# print("Number of gaps in disparities: ", ct_disp)

	# print("ranges: ", (data.range_min, data.range_max), data.angle_increment)
	
	for i in range(len(disparities)):
		# For dealing with nan values -- we may need to tweak this
		if math.isnan(ranges[i]):
			# extended_ranges[i] = 0 #ranges[i-1]
			# if (not math.isnan(ranges[i+1])):
			# 	extended_ranges[i] = ranges[i+1]
			# elif not math.isnan(ranges[i-1]):
			# 	extended_ranges[i] = ranges[i-1]
			# else:
			# extended_ranges[i] = extended_ranges[i-1]
			continue
		# If there is a gap defined after the right
		if disparities[i] > gap_threshold:
			# extension_length = min(100, (car_half_width + extender_padding) / 
			# 					(ranges[i]*math.sin(data.angle_increment)))
			# extension_length = min(50, math.atan((car_half_width + extender_padding)/ (ranges[i])) / data.angle_increment)
			# extension_length = min(50, math.atan2((car_half_width + extender_padding), (ranges[i])) / data.angle_increment)extended_ranges
			r = ranges[i]
			if not (data.range_min <= r <= data.range_max):
				continue
			# print("atan: ", math.atan((car_half_width + extender_padding) / (r)))
			# print("angles to overwrite: ", math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment, (r))
			extension_length = int(math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment) + 1
			if i+extension_length > len(ranges):
				extension_length = len(ranges)-i-1
			# extension_length = min(len(ranges)-i, extension_length+i)
			#print("{}: ext len".format(extension_length-i))
			# for j in range(i,int(extension_length)):
			# 	# if extended_ranges[j] >= ranges[i]:
			# 		# extended_ranges[j] = ranges[i]*math.cos(data.angle_increment) + ((j-i)*.001)
			# 		extended_ranges[j] = ranges[i] + ((j-extension_length+1)*.001)
			if np.nanmin(extended_ranges[i+1: i+extension_length+1]) >= r:
				extended_ranges[i: i+extension_length] = np.linspace(r+.0001, r+.001, num=extension_length)

		# If there is a gap defined after the left
		elif disparities[i] < -gap_threshold:
			# extension_length = min(100, (car_half_width + extender_padding) / 
			# 					(ranges[i+1]*math.sin(data.angle_increment)))
			# extension_length = min(50, math.atan((car_half_width + extender_padding)/(ranges[i+1])) / data.angle_increment)
			# extension_length = min(50, math.atan2((car_half_width + extender_padding), (ranges[i+1])) / data.angle_increment)
			r = ranges[i+1]
			if not (data.range_min <= r <= data.range_max):
				continue
			# print("atan: ", math.atan((car_half_width + extender_padding) / (r)))
			# print("angles to overwrite: ", math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment, (r))
			extension_length = int(math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment) + 1
			# extension_length = max(0, i+1-extension_length)
			if i-extension_length < 0:
				extension_length = i
			#print("{}: ext len".format(i+1-extension_length))
			# for j in range(int(extension_length),i+1):
			# 	# if extended_ranges[j] >= ranges[i+1]:
			# 		# extended_ranges[j] = ranges[i+1]*math.cos(data.angle_increment) + ((i+1-j)*.001)
			# 		extended_ranges[j] = ranges[i+1] + ((extension_length-j+1)*.001)
			if np.nanmin(extended_ranges[i-extension_length: i]) >= r:
				extended_ranges[i-extension_length+1: i+1] = np.linspace(r+.001, r+.0001, num=extension_length)
	# print("ranges: {}".format(ranges[90:-90]))
	# print("extended: {}".format(extended_ranges[90:-90]))	
	# padding = np.array([0]*90)
	# return np.append(padding, np.append(extended_ranges[90:-90], padding))
	# return extended_ranges[90:-90]
	# print("extended_ranges: ", np.amin(extended_ranges), np.amax(extended_ranges), np.sum(extended_ranges==np.inf), np.sum(extended_ranges==np.nan))
	# print(extended_ranges)
	# ct = 0
	# for i in range(len(extended_ranges)):
	# 	if math.isnan(extended_ranges[i]):
	# 		ct += 1
	# 		# extended_ranges[i] = 10
	# print("Number of nans in extended_ranges after compute: ", ct)
	return extended_ranges
	# return ranges

def callback(data):	
	scan = data
	scan.ranges = computeExtendedRanges(data)
	scan.header.frame_id = "car_3_laser"

	pub.publish(scan)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparity_extender',anonymous = True)
	rospy.Subscriber("/car_3/scan",LaserScan,callback)
	rospy.spin()
