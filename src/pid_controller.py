#! /usr/bin/env python3

import rospy
import numpy
import message_filters
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32

curr_time = 0.0
prev_time = 0.0
i_error = 0
prev_error = 0
mid_val = 0
d_error = 0.0
kp = 0.15355
ki = 0.00
kd = 10.5
car_speed = rospy.get_param("Vmax")

steer_val = 0

def callback(odom_data):

    global curr_time, prev_time, i_error, prev_error, kp, kd, ki, d_error, mid_val, steer_val, car_speed

    while rospy.get_time() == 0:
        print("Clock not yet published")
    
    #prev_time = rospy.get_time()

    curr_x = odom_data.pose.pose.position.x
    curr_y = odom_data.pose.pose.position.y

    drive_publisher = rospy.Publisher("/car_1/command", AckermannDrive, queue_size = 2)
    drive = AckermannDrive()

    # Finding the Error based on car current y

    if curr_x <= -1.0 or curr_x > 200.0:
        # STOP
        print("STOPPING THE CAR")
        drive.speed = 0.0
        drive_publisher.publish(drive)
        return

    elif curr_x <= 15.0:
        print("Car at < 15")
        ref_y = 0.0
        error = -(curr_y - ref_y)

    elif curr_x <= 30.0:
        print("Car at < 30")
        ref_y = 2.0
        error = -(curr_y - ref_y)

    elif curr_x <= 45.0:
        print("Car at < 45")
        ref_y = 5.0
        error = -(curr_y - ref_y)

    elif curr_x <= 60.0:
        print("Car at < 60")
        ref_y = 10.0
        error = -(curr_y - ref_y)

    elif curr_x <= 200.0:
        print("Car at < 200")
        ref_y = 15.0
        error = -(curr_y - ref_y)
    
    # Calculate derivative and Integral of the error
    
    curr_time = rospy.get_time()
    #print("Current Time is", curr_time - prev_time)

    # Derivative Component
    # if curr_time - prev_time >= 0.0001:
    #     d_error = (error - prev_error) / (curr_time - prev_time)

    #     prev_time = curr_time

    d_error = error - prev_error

    # Integral Component
    i_error += error

    # Saving the previous error
    prev_error = error

    # Total PID Error
    total_error = (kp * error) + (kd * d_error) + (ki * i_error)

    print("Total Error is", total_error)

 
    drive.speed = car_speed
    steer_val =  (kp * error) + (kd * d_error) + (ki * i_error)
    drive.steering_angle = steer_val

   
    # From -25 degrees to + 25 degrees

    if drive.steering_angle < -0.43:
        drive.steering_angle = -0.43
    if drive.steering_angle > 0.43:
        drive.steering_angle = 0.43

    #drive.steering_angle = 0.01 * error

    drive_publisher.publish(drive)

    total_error_publisher = rospy.Publisher("/total_error", Float32, queue_size = 10)
    total_error_publisher.publish(total_error)

    # derivative_error_publisher = rospy.Publisher("/error/d", Float32, queue_size = 10)
    # derivative_error_publisher.publish(total_error)

    # integral_error_publisher = rospy.Publisher("/error/i", Float32, queue_size = 10)
    # integral_error_publisher.publish(total_error)

    # proportional_error_publisher = rospy.Publisher("/error/p", Float32, queue_size = 10)
    # proportional_error_publisher.publish(total_error)







if __name__ == '__main__':
    
    # Do something

    rospy.init_node("pid_controller")
    odom_sub = message_filters.Subscriber("/car_1/base/odom", Odometry)

    ts = message_filters.TimeSynchronizer([odom_sub], 10)
    ts.registerCallback(callback)

    while not rospy.is_shutdown():

        rospy.spin()