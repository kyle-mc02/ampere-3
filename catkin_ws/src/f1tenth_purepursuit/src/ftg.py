#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan

pub = rospy.Publisher('ftg', AckermannDrive, queue_size=10)

gap_threshold = .15 # Threshold for determining if there is a disparity / gap between two ranges
car_half_width = .15 # The half width of the car for extending th esmall error 
extender_padding = 0 #0.2 # How far beyond car_half_width the extender will project ranges. 
kp = 80.0 

def computeExtendedRanges(data):
	# print('len(data.ranges) original: {}'.format(len(data.ranges)))
	# data: single message from topic /scan
	# angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
	# Outputs length in meters to object with angle in lidar scan field of view
	# Make sure to take care of NaNs etc.

    ranges = np.array(data.ranges)
    extended_ranges = np.array(ranges)
    disparities = np.diff(ranges)
    
    for i in range(len(disparities)):
        # For dealing with nan values -- we may need to tweak this
        if math.isnan(ranges[i]):
            extended_ranges[i] = 0
            continue
        # If there is a gap defined after the right
        if disparities[i] > gap_threshold:
            # extension_length = min(100, (car_half_width + extender_padding) / 
            # 					(ranges[i]*math.sin(data.angle_increment)))
            extension_length = min(50, math.atan((car_half_width + extender_padding)/ (ranges[i])) / data.angle_increment)
            # extension_length = math.atan2((car_half_width + extender_padding), (ranges[i])) / data.angle_increment
            extension_length = min(len(ranges), extension_length+i)
            for j in range(i,int(extension_length)):
                if extended_ranges[j] >= ranges[i]:
                    extended_ranges[j] = ranges[i]*math.cos(data.angle_increment)# + ((j-extension_length+1)*.004)
        # If there is a gap defined after the left
        elif disparities[i] < -gap_threshold:
            # extension_length = min(100, (car_half_width + extender_padding) / 
            # 					(ranges[i+1]*math.sin(data.angle_increment)))
            extension_length = min(50, math.atan((car_half_width + extender_padding)/ (ranges[i+1])) / data.angle_increment)
            # extension_length = math.atan2((car_half_width + extender_padding), (ranges[i+1])) / data.angle_increment
            extension_length = max(0, i+1-extension_length)
            for j in range(int(extension_length),i+1):
                if extended_ranges[j] >= ranges[i+1]:
                    extended_ranges[j] = ranges[i+1]*math.cos(data.angle_increment)# + ((extension_length-j+1)*.004)
    return extended_ranges[90:-90]

def find_gap(data):
	ranges = np.array(data)
	#diffs = np.diff(ranges)
	#diffs = np.absolute(diffs)
	deepest = 0
	deepest_ind = -1
	# end_ind = -1
	for i in range(1,len(ranges)):
		if math.isnan(ranges[i]):
			continue
		if (ranges[i] > deepest) and (not math.isinf(ranges[i])) and (not math.isnan(ranges[i])): # and data[i] < 4:
			deepest = ranges[i]
			deepest_ind = i
	return (deepest_ind)# + end_ind) //2

def callback(data):
    ranges = computeExtendedRanges(data)
    ranges_ind = find_gap(ranges)
    middle_index = len(ranges) // 2
    gap_offset = ranges_ind - middle_index
    error = gap_offset * data.angle_increment 
    angle = kp * error
    command = AckermannDrive()
    command.steering_angle = angle
    MAX_SPEED = 70
    command.speed = 5 * MAX_SPEED / (1 + .1 * abs(angle))
    pub.publish(command)

if __name__ == '__main__':
	rospy.init_node('follow_the_gap',anonymous = True)
	rospy.Subscriber("/car_3/scan",LaserScan,callback)
	rospy.spin()
