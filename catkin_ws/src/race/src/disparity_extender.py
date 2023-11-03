#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import copy

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('extended_ranges', LaserScan, queue_size=10)

gap_threshold = .2 # Threshold for determining if there is a disparity / gap between two ranges
car_half_width = .1 # The half width of the car for extending th esmall error 
extender_padding = 0.05 # How far beyond car_half_width the extender will project ranges. 

def computeExtendedRanges(data):
	# data: single message from topic /scan
	# # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
	# Outputs length in meters to object with angle in lidar scan field of view
	# Make sure to take care of NaNs etc.

	ranges = np.array(data.ranges)
	extended_ranges = ranges.copy()
	disparities = np.diff(ranges)
	
    for i in range(len(disparities)) :
		# If there is a gap defined after the right
        if disparities[i] > gap_threshold:
			extension_length = ((car_half_width + extender_padding) / 
								ranges[i]*math.cos(data.angle_increment))
			extension_length = min(len(ranges), extension_length+i)
			extended_ranges[i:(i+extension_length)] = ranges[i]
		# If there is a gap defined after the left
        elif disparities[i] < -gap_threshold:
            extension_length = ((car_half_width + extender_padding) / 
								ranges[i+1]*math.cos(data.angle_increment))
			extension_length = max(0, i+1-extension_length)
			extended_ranges[(i+1-extension_length):i] = ranges[i+1]
			
    return extended_ranges

def callback(data):	
	scan = copy.deepcopy(data)
	scan.ranges = computeExtendedRanges(data)

	pub.publish(scan)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparity_extender',anonymous = True)
	rospy.Subscriber("/car_3/scan",LaserScan,callback)
	rospy.spin()
