#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import rospkg
from os import path

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
file_path = r.get_path("lab3")
tracker = path.join(file_path, 'lab3_track.csv')

array_publisher = rospy.Publisher("/reference_plot_2", MarkerArray, queue_size = 2)

rospy.init_node("reference_plot")


def plotreference(data, array_publisher):

    markerarray = MarkerArray()

    for idr, points in enumerate(data):
        
        
        marker = Marker()

        marker.header.frame_id = "odom"
        marker.id = idr

        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        marker.pose.position.x, marker.pose.position.y = points[1], points[2]


        markerarray.markers.append(marker)

    
    
    array_publisher.publish(markerarray)




if __name__ == '__main__':
    
    rate = rospy.Rate(25)

    data = np.loadtxt(tracker, skiprows = 1, dtype = float, delimiter = ',')
    
    

    while not rospy.is_shutdown():

        plotreference(data, array_publisher)

        rate.sleep()
