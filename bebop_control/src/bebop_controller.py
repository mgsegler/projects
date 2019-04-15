#! /usr/bin/env python

from __future__ import print_function
import bebop_states
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Transform
import tf.transformations
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
import time
import math

button1 = 0
button2 = 0
button3 = 0
button4 = 0

operating = 0

rospy.init_node('BebopCon', anonymous=True)

def getJoy(joy_state):
	global button1
	button1 = joy_state.buttons[0]
	global button2
	button2 = joy_state.buttons[1]
	global button3
	button3 = joy_state.buttons[2]
	global button4
	button4 = joy_state.buttons[3]

rospy.init_node('BebopCon', anonymous=True)

bebop1 = bebop.Bebop()
bebop2 = bebop.Bebop()

takeoff = Empty()
land = Empty()
emergency = Empty()

takeoff_pub_1 = rospy.Publisher('/bebop1/takeoff', Empty, queue_size = 0)
land_pub_1 = rospy.Publisher('/bebop1/land', Empty, queue_size = 5)
emergency_pub_1 = rospy.Publisher('/bebop/1reset', Empty, queue_size = 5)
bebop_cmd_pub_1 = rospy.Publisher('/bebop1/cmd_vel', Twist, queue_size=0)

takeoff_pub_2 = rospy.Publisher('/bebop2/takeoff', Empty, queue_size = 0)
land_pub_2 = rospy.Publisher('/bebop2/land', Empty, queue_size = 5)
emergency_pub_2 = rospy.Publisher('/bebop2/reset', Empty, queue_size = 5)
bebop_cmd_pub_2 = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size=0)

rospy.Subscriber('/joy', Joy, getJoy)
rospy.Subscriber('vrpn_client_node/Bebop1/pose', PoseStamped, bebop1.getPose)
rospy.Subscriber('vrpn_client_node/Bebop2/pose', PoseStamped, bebop2.getPose)
rate = rospy.Rate(100)

start_time = time.time()
bebop1.setGoal(-0.75,-0.75,1,0)
bebop2.setGoal(0.75,0.75,1,0)
while not rospy.is_shutdown():
	elapsedTime = time.time() - start_time

	#Overloads setCmdVel to provide command velocities to ( linear x, y, z and angular z)
	#Valid values = [-1 1]
	'''
	bebop1.setCmdVel(0,0,0,0.5)
	bebop_cmd_pub.publish(bebop1.control_cmd)
	'''
	'''
	print('Bebop1')
	bebop1.Update()
	print('Bebop2')
	bebop2.Update()
	'''
	if(elapsedTime > 2):
		op_start_time = time.time()
		while(operating == 1):
			elapsedTime = time.time() - op_start_time

			'''
			if(elapsedTime >= 0 and elapsedTime < 20):
				bebop1.setGoal(0.2, 0, 1, -90.0)
			if(elapsedTime >= 20):
				bebop1.setGoal(-0.2, 0, 1, -90.0)
			'''
			print('Bebop1')
			bebop1.Update()
			print('Bebop2')
			bebop2.Update()
			bebop_cmd_pub_1.publish(bebop1.control_cmd)
			bebop_cmd_pub_1.publish(bebop1.control_cmd)

			if(button2==1):
				operating = 0
				land_pub_1.publish(land)
				land_pub_2.publish(land)

			if(button3==1):
				emergency_pub_1.publish(emergency)
				emergency_pub_2.publish(emergency)
				quit()
			rate.sleep()
		if(button1==1):
			takeoff_pub_1.publish(takeoff)
			takeoff_pub_2.publish(takeoff)
			operating = 1
		if(operating == 0):
			elapsedTime = 0

		rate.sleep()
