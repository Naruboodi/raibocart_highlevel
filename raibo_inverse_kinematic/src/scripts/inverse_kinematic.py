#!/usr/bin/env python3

import sys
import rospy
import math

from raibo_msgs.msg import speed_sp
from geometry_msgs.msg import Twist

v_car_x_kmh = 0.0
v_car_x_mps = 0.0
v_steering = 0.0
sp_output = speed_sp()
def callback(data):
    # Assign cmd_vel parameter as m/s
    linear_x = data.linear.x    #m/s
    linear_y = data.linear.y
    linear_z = data.linear.z

    angular_x = data.angular.x
    angular_y = data.angular.y
    angular_z = data.angular.z   #rad/s

    rospy.loginfo(data)

    if linear_x != 0 or linear_y != 0:
        v_car_x_mps = linear_x/(math.cos(angular_z))
        v_steering = math.atan((1.0985 * angular_z) / v_car_x_mps) * (180/math.pi)  #deg
        # End to end dis = 1.46685
        # Front wheel: 13x5 inch   dia 0.3302/2 = 0.1651 m
        # Back wheel: 16x7 inch      dia 0.4064/2 = 0.2032 m
        # Frame lenght: 1.0985 m
        v_car_x_kmh = v_car_x_mps * 3.6     #kmh
        
    if linear_x == 0 and linear_y == 0 and angular_z != 0:
        v_car_x_kmh = 0
        v_steering = math.atan((1.0985 * angular_z) / 1) * (180/math.pi)  #deg

    if linear_x == 0 and linear_y == 0 and angular_z == 0:
        v_car_x_kmh = 0
        v_steering = 0

    print(v_car_x_kmh, v_steering)
    sp_output.bldc_speed_sp = v_car_x_kmh
    sp_output.steering_speed_sp = v_steering


    # Inverse kinematic solution


def inverseKinematic():
    # initial ROS publisher
    speed_sp_pub = rospy.Publisher('/micro/speed_sp', speed_sp, queue_size=100)

    # initial ROS node
    rospy.init_node('raibocart_inverseKinematic_node')

    #rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # initial ROS subcriber
        #rospy.Subscriber("cmd_vel", Twist, callback)
        velocity = rospy.wait_for_message("cmd_vel", Twist, timeout=None)
        callback(velocity)
        # Run away

        # publish speed bldc & steering 
        rospy.loginfo(sp_output)
        speed_sp_pub.publish(sp_output)
        #rate.sleep()

if __name__ == '__main__':
    try:
        inverseKinematic()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")
