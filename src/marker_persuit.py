#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from std_msgs.msg import Float32

array_publisher = rospy.Publisher("/reference_plot_2", MarkerArray, queue_size = 2)
markerarray = MarkerArray()

rospy.init_node("reference_plot")

def plotreference(markerarray, data, array_publisher):

    for id, points in enumerate(data):
        marker = Marker()
        marker.header.frame_id = "odom"

        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        marker.pose.position.x, marker.pose.position.y = points[1], points[2]

        markerarray.markers.append(marker)

        array_publisher.publish(markerarray)


if __name__ == '__main__':
    
    rate = rospy.Rate(1.0)

    data = np.loadtxt("/home/cse4568/catkin_ws/src/lab3/lab3_track.csv", skiprows = 1, dtype = float, delimiter = ',')


    while not rospy.is_shutdown():
        #rospy.Subscriber("/car_1/base/odom", Odometry, callback = get_frameid)
        plotreference(markerarray, data, array_publisher)
        
        rate.sleep()