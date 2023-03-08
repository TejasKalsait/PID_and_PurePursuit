#! /usr/bin/env python3

import tf
import numpy as np
import rospy
import message_filters

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

curr_heading = None
curr_y = None
curr_x = None
l_dist = 30.0       # Lookahead dist
l_point = None      # Lookahead point
min_dist = 1000
which_marker = None


def main_callback(odom_msg, ref_msg):
    
    global curr_heading, curr_y, curr_x, l_dist, l_point, min_dist, which_marker

    tolerance = 1.0

    #print("Entered Main callback")

    # Current Position of the Car
    curr_y = odom_msg.pose.pose.position.y
    curr_x = odom_msg.pose.pose.position.y


    all_markers = ref_msg.markers
    print("Length is", len(all_markers))
    
    for imarker in all_markers:

        # imarker is of type Marker

        #print("Executing For Loop")

        marker_x = imarker.pose.position.x
        marker_y = imarker.pose.position.y

        #curr_dist = np.linalg.norm(np.array(curr_x, curr_y) - np.array(marker_x, marker_y))
        curr_dist = np.sqrt(np.power((marker_x - curr_x), 2) + np.power((marker_y - curr_y), 2))
        #print(curr_dist)

        min_dist = min(min_dist, curr_dist)
        
        if curr_dist == min_dist:
            which_marker = imarker.id

        
    
    # Now, which_marker contains the ID of the closent marker and min_dist is how far away 

    print("ID of the closest marker is", which_marker, "and distance is", min_dist)

    # Markers strting from closest point to until lookahead distance

    for look_marker in all_markers:

        if look_marker.id > which_marker:
            # Starting with first marker and ending with 2*ld
            look_marker_x2 = np.power(look_marker.pose.position.x, 2)
            look_marker_y2 = np.power(look_marker.pose.position.y, 2)

            if look_marker_x2 + look_marker_y2 == np.power(l_dist, 2) + tolerance or look_marker_x2 + look_marker_y2 == np.power(l_dist, 2) - tolerance:
                    l_point = look_marker
                    print("Just hope this is fucking l_point")



        if look_marker.id > which_marker + (2* l_dist):
            break




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

    while not rospy.is_shutdown():

        rospy.spin()