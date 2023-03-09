#! /usr/bin/env python3

from tf.transformations import euler_from_quaternion
import numpy as np
import rospy
import message_filters
import math

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PointStamped

# /clicked_point
wheel_dist = 0.384

curr_heading = None
curr_y = None
curr_x = None
speed = 4.0
kdd = 0.5
l_dist_min = 2.75
l_dist_max = 8.0
l_dist = np.clip(kdd * speed, l_dist_min, l_dist_max)       # Lookahead dist
print("The Look Distance is", l_dist)
l_point = None      # Lookahead point
min_dist = float('inf')
which_marker = None
curr_heading = None
goal_heading = None
curr_dist = None



def main_callback(odom_msg, ref_msg):
    
    global curr_heading, curr_y, curr_x, l_dist, l_point, min_dist, curr_heading, goal_heading, which_marker, curr_dist, wheel_dist, speed, min_dist, prev_l_point

    tolerance = 4.0

    #print("Entered Main callback")
    #rospy.sleep(1.0)

    # Current Position of the Car
    curr_y = odom_msg.pose.pose.position.y
    curr_x = odom_msg.pose.pose.position.x

    #print("Current X of the car is", curr_x)


    all_markers = ref_msg.markers
    #print("Length is", len(all_markers))
    
    for imarker in all_markers:

        # imarker is of type Marker
        #print("Checking current marker", imarker.id)
        #print("Executing For Loop")

        # marker_x = imarker.pose.position.x
        # marker_y = imarker.pose.position.y

        #curr_dist = np.linalg.norm(np.array(curr_x, curr_y) - np.array(marker_x, marker_y))
        curr_dist = np.sqrt(np.power((imarker.pose.position.x - curr_x), 2) + np.power((imarker.pose.position.y - curr_y), 2))
        #print("Curr",curr_dist)
        if curr_dist < min_dist:
            min_dist = curr_dist
            which_marker = imarker.id
        #print("Min", min_dist)

        
    
    # Now, which_marker contains the ID of the closent marker and min_dist is how far away 
    #print("Car X is", curr_x)
    print("ID of the closest marker is", which_marker, "and distance is", min_dist)

    # Markers strting from closest point to until lookahead distance

    for look_marker in all_markers:

        if look_marker.id > which_marker:
            #print("starting with marker", look_marker.id)
            # Starting with first marker and ending with 3*ld
            look_marker_x2 = np.power(look_marker.pose.position.x - curr_x, 2)
            look_marker_y2 = np.power(look_marker.pose.position.y - curr_y, 2)

            #print(look_marker_x2 + look_marker_y2)
            #print(np.power(l_dist, 2))

            if look_marker_x2 + look_marker_y2 <= np.power(l_dist, 2) + tolerance and look_marker_x2 + look_marker_y2 >= np.power(l_dist, 2) - tolerance:
                    l_point = look_marker
                    prev_l_point = l_point
                    #print("Look_Ahead point is", l_point.id)
                    break
  
        if look_marker.id > which_marker + (3 * l_dist):
            l_point = prev_l_point
            break

    #Current Heading

    min_dist = float('inf')
    curr_dist = None
    which_marker = None  

    quater_x = odom_msg.pose.pose.orientation.x
    quater_y = odom_msg.pose.pose.orientation.y
    quater_z = odom_msg.pose.pose.orientation.z
    quater_w = odom_msg.pose.pose.orientation.w

    curr_quat_msg = euler_from_quaternion([quater_x, quater_y, quater_z, quater_w])

    curr_heading = curr_quat_msg[2] + math.pi

    #Yaw is curr_heading

    #print("Current Heading is", curr_heading)

    # GOAL HEADING

    dx = curr_x - l_point.pose.position.x
    dy = curr_y - l_point.pose.position.y

    # For point clicked

    # dx = curr_x - point_msg.point.x
    # dy = curr_y - point_msg.point.y

    goal_heading = math.atan2(dy, dx) % (2 * math.pi)

    #print("Goal Heading is", goal_heading)

    turn_angle = (goal_heading - curr_heading + 3*math.pi) % (2*math.pi) - math.pi

    if turn_angle > 0.43:
        turn_angle = 0.43
    elif turn_angle < -0.43:
        turn_angle = -0.43

    distance = ((odom_msg.pose.pose.position.x - l_point.pose.position.x)**2 + \
                    (odom_msg.pose.pose.position.y - l_point.pose.position.y)**2)**0.5
    
    print(f"current: {curr_heading}, goal: {goal_heading}, change: {turn_angle}")
    #print("Steering angle is", turn_angle)
    
    drive_publisher = rospy.Publisher("/car_1/command", AckermannDrive, queue_size = 10)
    drive = AckermannDrive()
    drive.speed = speed
    drive.steering_angle = turn_angle

    if distance < 0.5:
        drive.speed = 0
        drive.steering_angle = 0.0

    drive_publisher.publish(drive)
    

def pp_controller_def():
    rospy.init_node("pid_controller")
    # rospy.Subscriber("/car_1/base/odom", Odometry, main_callback)
    # rospy.Subscriber("/reference_plot_2", MarkerArray, ref_callback)

    odom_sub = message_filters.Subscriber("/car_1/base/odom", Odometry)
    ref_sub = message_filters.Subscriber("/reference_plot_2", MarkerArray)
    #point_sub = message_filters.Subscriber("/clicked_point", PointStamped)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, ref_sub], queue_size = 2, slop = 1, allow_headerless = True)
    #ts = message_filters.TimeSynchronizer([odom_sub, ref_sub], queue_size = 2)
    print("TimeSync Done, Registerning callback")
    ts.registerCallback(main_callback)

if __name__ == '__main__':
    
    # Do something

    pp_controller_def()
    rospy.spin()
