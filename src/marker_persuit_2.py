#! /usr/bin/env python3


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np

rospy.init_node("reference_plot")
publisher = rospy.Publisher("/reference_plot_2", Marker, queue_size = 10)
marker = Marker()


def plotreference(publisher, marker, data):

    offset = 0.0

    marker.header.frame_id = "odom"

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.points = []

    marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1

    # Yellow combinationwith alpha 1

    marker.color.a = 1.0
    marker.color.b = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0



    # marker.pose.position.x = 0.0 + offset
    # marker.pose.position.y = 0.0 + offset
    # marker.pose.position.z = 0.0 + offset

    marker.pose.orientation.x = 0.0 + offset
    marker.pose.orientation.y = 0.0 + offset
    marker.pose.orientation.z = 0.0 + offset
    marker.pose.orientation.w = 1.0 + offset

    for idr, marks in enumerate(data):

        pointer = Point()

        pointer.x = marks[1]
        pointer.y = marks[2]
        pointer.z = 0.0

        marker.points.append(pointer)

    

    # Publishing the Marker 
    publisher.publish(marker)



if __name__ == '__main__':
    
    
    rate = rospy.Rate(1.0)

    data = np.loadtxt("/home/cse4568/catkin_ws/src/lab3/lab3_track.csv", skiprows = 1, dtype = float, delimiter = ',')

    while not rospy.is_shutdown():
        #rospy.Subscriber("/car_1/base/odom", Odometry, callback = get_frameid)
        plotreference(publisher, marker, data)
        rate.sleep()