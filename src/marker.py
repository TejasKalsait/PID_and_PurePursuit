#! /usr/bin/env python3

# Part 1 is to plot the reference line to rviz
# I plan to publish the marker message to a topic named /reference_line
# Then make the rviz display this topic

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

rospy.init_node("reference_plot")
publisher = rospy.Publisher("/reference_line", Marker, queue_size = 10)
marker = Marker()

my_head= None

def get_frameid(data):
    global my_head
    my_head = data.header

def plotreference(publisher, marker):


    
    #rospy.sleep(1.0)
    #print(my_head)

    offset = 0
    
    marker.header.frame_id = "odom"
    #print(marker.header.frame_id)
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.points = []

    marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1

    # Yellow combinationwith alpha 1

    marker.color.a = 1.0
    marker.color.b = 0.0
    marker.color.r = 1.0
    marker.color.g = 1.0

    marker

    marker.pose.position.x = 0.0 + offset
    marker.pose.position.y = 0.0 + offset
    marker.pose.position.z = 0.0 + offset

    marker.pose.orientation.x = 0.0 + offset
    marker.pose.orientation.y = 0.0 + offset
    marker.pose.orientation.z = 0.0 + offset
    marker.pose.orientation.w = 1.0 + offset

    zeroth_x, zeroth_y, zeroth_z = 0.0 + offset, 0.0 + offset, 0.0 + offset
    first_x , first_y, first_z = 15.0 + offset, 0.0 + offset, 0.0 + offset
    second_x , second_y, second_z = 15.0 + offset, 2.0 + offset, 0.0 + offset
    third_x , third_y, third_z = 30.0 + offset, 2.0 + offset, 0.0 + offset
    forth_x , forth_y, forth_z = 30.0 + offset, 5.0 + offset, 0.0 + offset
    fifth_x , fifth_y, fifth_z = 45.0 + offset, 5.0 + offset, 0.0 + offset
    sixth_x , sixth_y, sixth_z = 45.0 + offset, 10.0 + offset, 0.0 + offset
    seventh_x , seventh_y, seventh_z = 60.0 + offset, 10.0 + offset, 0.0 + offset
    eightth_x , eightth_y, eightth_z = 60.0 + offset, 15.0 + offset, 0.0 + offset
    ninth_x , ninth_y, ninth_z = 200.0 + offset, 15.0 + offset, 0.0 + offset

    # Zeroth point
    pointer = Point()
    pointer.x, pointer.y, pointer.z = zeroth_x, zeroth_y, zeroth_z
    marker.points.append(pointer)
    
    # First point
    pointer = Point()
    pointer.x, pointer.y, pointer.z = first_x, first_y, first_z
    marker.points.append(pointer)

    # Second Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = second_x, second_y, second_z
    marker.points.append(pointer)

    # Third Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = third_x, third_y, third_z
    marker.points.append(pointer)

    # Forth Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = forth_x, forth_y, forth_z
    marker.points.append(pointer)

    # Fifth Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = fifth_x, fifth_y, fifth_z
    marker.points.append(pointer)

    # Sixth Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = sixth_x, sixth_y, sixth_z
    marker.points.append(pointer)

    # seventh Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = seventh_x, seventh_y, seventh_z
    marker.points.append(pointer)

    # eightth Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = eightth_x, eightth_y, eightth_z
    marker.points.append(pointer)

    # ninth Pointer
    pointer = Point()
    pointer.x, pointer.y, pointer.z = ninth_x, ninth_y, ninth_z
    marker.points.append(pointer)

    # Publishing the Marker 
    publisher.publish(marker)



if __name__ == '__main__':
    
    
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        #rospy.Subscriber("/car_1/base/odom", Odometry, callback = get_frameid)
        plotreference(publisher, marker)
        rate.sleep()