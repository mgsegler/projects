#!/usr/bin/python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

pos_x = 0.0
pos_y = 0.0
pos_z = 0.0
yaw = 0.0

log_file = open('bebop_log.dat', 'w')
log_file.truncate()

def getPose(data):
    global pos_x
    pos_x = truncate(data.pose.position.x, 3)
    global pos_y
    pos_y = truncate(data.pose.position.z, 3)	#Optitrack uses XZY coordinates
    global pos_z
    pos_z = truncate(data.pose.position.y, 3)
    ori_x = truncate(data.pose.orientation.x, 3)
    ori_y = truncate(data.pose.orientation.z, 3)
    ori_z = truncate(data.pose.orientation.y, 3)
    ori_w = truncate(data.pose.orientation.w, 3)

    bebop_quat = [ori_x, ori_y, ori_z, ori_w]
    (roll, pitch, yaw_rad) = euler_from_quaternion(bebop_quat)
    global yaw
    yaw = truncate(yaw_rad*(180/3.14), 3)

    str_val = '{}, {}, {}, {}\n'.format(pos_x, pos_y, pos_z, yaw)
    log_file.write(str_val)

def truncate(number, digits):
    stepper = pow(10.0, digits)
    return math.trunc(stepper * number) / stepper

rospy.init_node('bebop_logger', anonymous=True)
rospy.Subscriber('vrpn_client_node/Bebop1/pose', PoseStamped, getPose)

rospy.spin()
