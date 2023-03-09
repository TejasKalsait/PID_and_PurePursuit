#! /usr/bin/env python3

from tf.transformations import euler_from_quaternion
import numpy as np
import rospy
import message_filters
import math

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDrive

# /clicked_point

curr_heading = None
curr_y = None
curr_x = None
l_dist = 3.0       # Lookahead dist
l_point = None      # Lookahead point
min_dist = 1000
which_marker = None
curr_heading = None
goal_heading = None


def main_callback(odom_msg, ref_msg):
    
    global curr_heading, curr_y, curr_x, l_dist, l_point, min_dist, which_marker, curr_heading, goal_heading

    tolerance = 2.0

    #print("Entered Main callback")
    #rospy.sleep(1.0)

    # Current Position of the Car
    curr_y = odom_msg.pose.pose.position.y
    curr_x = odom_msg.pose.pose.position.x


    all_markers = ref_msg.markers
    #print("Length is", len(all_markers))
    
    for imarker in all_markers:

        # imarker is of type Marker
        #print("Checking current marker", imarker.id)
        #print("Executing For Loop")

        marker_x = imarker.pose.position.x
        marker_y = imarker.pose.position.y

        #curr_dist = np.linalg.norm(np.array(curr_x, curr_y) - np.array(marker_x, marker_y))
        curr_dist = np.sqrt(np.power((marker_x - curr_x), 2) + np.power((marker_y - curr_y), 2))
        #print("Curr",curr_dist)

        min_dist = min(min_dist, curr_dist)
        #print(min_dist)
        
        if curr_dist == min_dist:
            which_marker = imarker.id

        
    
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
                    #print("Look_Ahead point is", l_point.id)
                    break
  
        if look_marker.id > which_marker + (2 * l_dist):
            break

    # Current Heading

    quater_x = odom_msg.pose.pose.orientation.x
    quater_y = odom_msg.pose.pose.orientation.y
    quater_z = odom_msg.pose.pose.orientation.z
    quater_w = odom_msg.pose.pose.orientation.w

    curr_quat_msg = euler_from_quaternion([quater_x, quater_y, quater_z, quater_w])
    curr_heading = curr_quat_msg[2] + np.pi
    # Yaw is curr_heading
    #print("Current Heading is", curr_heading)

    # Goal Heading

    goal_quat_msg = math.atan2(l_point.pose.position.y, l_point.pose.position.x)

    #print(goal_quat_msg)

    goal_heading = goal_quat_msg % (2 * np.pi)

    #print("Goal Heading is", goal_heading)
    
    #print("#$%#$$$$$$$$$$#%$%$#%$#%$", curr_heading - goal_heading)
    
    # drive_publisher = rospy.Publisher("/car_1/command", AckermannDrive, queue_size = 10)
    # drive = AckermannDrive()
    # drive.speed = 4.0
    # drive.steering_angle = 1 * (goal_heading - curr_heading)

    # drive_publisher.publish(drive)
    




if __name__ == '__main__':
    
    # Do something

    rospy.init_node("pid_controller")
    # rospy.Subscriber("/car_1/base/odom", Odometry, main_callback)
    # rospy.Subscriber("/reference_plot_2", MarkerArray, ref_callback)

    odom_sub = message_filters.Subscriber("/car_1/base/odom", Odometry)
    ref_sub = message_filters.Subscriber("/reference_plot_2", MarkerArray)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, ref_sub], queue_size = 2, slop = 1, allow_headerless = True)
    #print("TimeSync Done, Registerning callback")
    ts.registerCallback(main_callback)

    try:

        while not rospy.is_shutdown():

            rospy.spin()
    except rospy.ROSInterruptException:

        drive_publisher = rospy.Publisher("/car_1/command", AckermannDrive, queue_size = 10)
        drive = AckermannDrive()
        drive.speed = 0.0
        drive.steering_angle = 0.1 * (curr_heading -  goal_heading)
        print("STOPPING")
        drive_publisher.publish(drive)