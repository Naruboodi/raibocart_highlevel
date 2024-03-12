#!/usr/bin/env python3

import sys
import rospy
import math

from raibo_msgs.msg import speed_sp
from geometry_msgs.msg import Twist

v_car_x = 0.0
v_car_y = 0.0
v_steering = 0.0
sp_output = speed_sp()
def callback(data):
    # Assign cmd_vel parameter
    linear_x = data.linear.x
    linear_y = data.linear.y
    linear_z = data.linear.z

    angular_x = data.angular.x
    angular_y = data.angular.y
    angular_z = data.angular.z   

    rospy.loginfo(data)

    if linear_x != 0 or linear_y != 0:
        v_car_x = linear_x # kmh
        v_steering = angular_z # deg
    if linear_x == 0 and linear_y == 0 and angular_z != 0:
        v_car_x = 0
        v_steering = angular_z #deg

    if linear_x == 0 and linear_y == 0 and angular_z == 0:
        v_car_x = 0
        v_steering = 0

    print(v_car_x, v_steering)
    sp_output.bldc_speed_sp = v_car_x
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
