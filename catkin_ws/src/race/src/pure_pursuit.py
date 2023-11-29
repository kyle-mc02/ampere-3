#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
import tf

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = 'car_3'
trajectory_name     = 'race_line'

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq          = 0
control_polygon = PolygonStamped()

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('/home/maxwell/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))


def dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y


    # The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
    base_proj_ind = 0
    min_dist = 6

    for i in range(len(plan)):
        if path_resolution[i] < min_dist:
            base_proj_ind = i
            min_dist = path_resolution[i]

    
    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]
    

    # TODO: Tune both of these values
    lookahead_distance = 1.0
    tolerance = 0.01


    # Utilizing the base projection, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    target_point_ind = -1
    for i in range(base_proj_ind, len(plan)):
        if math.abs(dist(plan[i], plan[base_proj_ind]) - lookahead_distance) <= tolerance:
            target_point_ind = i
            break


    # Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.

    if target_point_ind == -1:
        return
    target_point = plan[target_point_ind]
    yt = math.abs(math.cos(heading)*(odom_y-target_point[1]) - math.sin(heading)*(odom_x-target_point[0]))
    alpha = math.asin(yt/lookahead_distance)
    wheel_angle = math.atan2((2*WHEELBASE_LEN*math.sin(alpha))/lookahead_distance)

    # Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle

    if wheel_angle > STEERING_RANGE:
        wheel_angle = STEERING_RANGE
    elif wheel_angle < -STEERING_RANGE:
        wheel_angle = -STEERING_RANGE
    command.steering_angle = wheel_angle

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    command.speed = 20.0
    command_pub.publish(command)

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

    # These are set to zero only so that the template code builds. 
    pose_x=0    
    pose_y=0
    target_x=0
    target_y=0


    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
