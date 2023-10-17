#!/usr/bin/env python
import math
import rospy
from collections import deque
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 30.0 #TODO
kd = 135.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
errors = deque(maxlen=200) # For how long in the past should error be accumulated?

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 35	#TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_3/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global ki
	global angle
	global errors

	print("PID Control Node is Listening to error")

	## Your PID code goes here

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	angle = (-kp * data.pid_error) + (-ki * sum(errors)) +  (-kd * (data.pid_error - prev_error))
	errors.append(data.pid_error)

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# Make sure the steering value is within bounds [-100,100]
	if angle < -100 :
		angle = -100
	elif angle > 100 :
		angle = 100
	command.steering_angle = angle

	# Make sure the velocity is within bounds [0,100]
	if vel_input < 0:
		vel_input = 0
	elif vel_input > 100:
		vel_input = 100
	command.speed = vel_input

	# Move the car autonomously
	print(command)
	command_pub.publish(command)

	prev_error = data.pid_error

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	global errors
	#kp = input("Enter Kp Value: ")
	#kd = input("Enter Kd Value: ")
	#ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
