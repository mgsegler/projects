#! /usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Transform
import tf.transformations
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
import time
import math

kp = 0.25
kp_z = 2
kd = 0
kd_z = 0
ki = 0.06
ki_z = 0.4

a = 0.0
b = 0.01
speed = 0.5

button1 = 0
button2 = 0
button3 = 0
button4 = 0

operating = 0

T = 0.01

class States:
	def __init__(self, posx, posy, posz, 
			   rotx, roty, rotz, velx, vely, velz, prevx, prevy, prevz):
		self.posx = posx
		self.posy = posy
		self.posz = posz
		self.rotx = rotx
		self.roty = roty
		self.rotz = rotz
		self.velx = velx
		self.vely = vely
		self.velz = velz
		self.prevx = prevx
		self.prevy = prevy
		self.prevz = prevz
		
class CmdVel:
	def __init__(self, cmdx, cmdy, cmdz, cmdrx, cmdry, cmdrz, cmdyaw):
		self.cmdx = cmdx
		self.cmdy = cmdy
		self.cmdz = cmdz
		self.cmdrx = cmdrx
		self.cmdry = cmdry
		self.cmdrx = cmdrz
		self.cmdyaw = cmdyaw
		
class Error:
	def __init__(self, ex, ey, ez, eyaw, velx, vely, velz, velyaw, abstx, absty, abstz, abstyaw):
		self.ex = ex
		self.ey = ey
		self.ez = ez
		self.eyaw = eyaw
		self.velx = velx
		self.vely = vely
		self.velz = velz
		self.velyaw = velyaw
		self.abstx = abstx
		self.abst = absty
		self.abstz = abstz
		self.abstyaw = abstyaw
		
		
	

rospy.init_node('BebopCon', anonymous=True)

#Initialize Bebop1 with default values
bebop1 = States(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
bebop1_goal = States(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
bebop1_cmd_vel = CmdVel(0.0,0.0,0.0,0.0,0.0,0.0,0.0)
bebop1_error = Error(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)

bebop1_control_cmd = Twist()
bebop1_control_cmd.linear.x = 0
bebop1_control_cmd.linear.y = 0
bebop1_control_cmd.linear.z = 0
bebop1_control_cmd.angular.x = 0.0
bebop1_control_cmd.angular.y = 0.0
bebop1_control_cmd.angular.z = 0.0

#TODO: Roll getPose into one class with States
def getPose(data):
	bebop1.posx = truncate(data.pose.position.x, 3)
	bebop1.posy = truncate(data.pose.position.z, 3)
	bebop1.posz = truncate(data.pose.position.y, 3)
	bebop1.rotx = truncate(data.pose.orientation.x,3)
	bebop1.roty = truncate(data.pose.orientation.z,3)
	bebop1.rotz = truncate(data.pose.orientation.y,3)
#	bebop1.rotw = data.pose.orientation.w
	
	#tf.Quaternion q(pose.transform.rotation.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w)
	#tf.Matrix3x3 m(q)
	#m.getRPY(bebop.pitch, bebop.roll, bebop.yaw_RAD)
	#bebop.yaw = bebop.yaw_RAD*(180/3.14);
	

	
def setGoal(goal_x,goal_y,goal_z):
	bebop1_goal.posx = goal_x
	bebop1_goal.posy = goal_y
	bebop1_goal.posz = goal_z
	print("Goal\nX: ", bebop1_goal.posx, " Y: ",bebop1_goal.posy, " Z: ", bebop1_goal.posz)

def getPosError():
	bebop1_error.ex = bebop1_goal.posx - bebop1.posx
	bebop1_error.ey = bebop1_goal.posy - bebop1.posy
	bebop1_error.ez = bebop1_goal.posz - bebop1.posz
	print("\nCurrent Position\n X: ", bebop1.posx, " Y: ", bebop1.posy, " Z: ", bebop1.posz)
	print("Position Error\nX: ", bebop1_error.ex, " Y: ",bebop1_error.ey, " Z: ", bebop1_error.ez)

def getVel():
	bebop1.velx = (bebop1.prevx - bebop1.posx)/T
	bebop1.vely = (bebop1.prevy - bebop1.posy)/T
	bebop1.velz = (bebop1.prevz - bebop1.posz)/T
	print("Velocity\nX: ", bebop1.velx, " Y: ",bebop1.vely, " Z: ", bebop1.velz)
	
def getAbst(a, b, prev_pos, current_pos):
	s = (prev_pos+current_pos)/2
	q = (b-a)
	return q*s

def getAbstError():
	bebop1_error.abstx = truncate(getAbst(a, b, bebop1.prevx, bebop1.posx), 3)
	bebop1_error.absty = truncate(getAbst(a, b, bebop1.prevy, bebop1.posy), 3)
	bebop1_error.abstz = truncate(getAbst(a, b, bebop1.prevz, bebop1.posz), 3)
	print("Abst Error\nX: ",bebop1_error.abstx, " Y: ",bebop1_error.absty, " Z: ", bebop1_error.abstz)
	
	
def setCmdVel():
	bebop1_cmd_vel.cmdx = (kp*bebop1_error.ex) - (kd*bebop1.velx) + (ki*bebop1_error.abstx)
	bebop1_cmd_vel.cmdy = -(kp*bebop1_error.ey) - (kd*bebop1.vely) + (ki*bebop1_error.absty)
	bebop1_cmd_vel.cmdz = (kp_z*bebop1_error.ez) - (kd_z*bebop1.velz) + (ki_z*bebop1_error.abstz)
	bebop1_control_cmd.linear.x = bebop1_cmd_vel.cmdx
	bebop1_control_cmd.linear.y = bebop1_cmd_vel.cmdy
	bebop1_control_cmd.linear.z = bebop1_cmd_vel.cmdz
	bebop1_control_cmd.angular.x = 0.0
	bebop1_control_cmd.angular.y = 0.0
	bebop1_control_cmd.angular.z = 0.0
	
	if(bebop1_control_cmd.linear.x > speed):
		bebop1_control_cmd.linear.x = speed
	if(bebop1_control_cmd.linear.y > speed):
		bebop1_control_cmd.linear.y = speed
	if(bebop1_control_cmd.linear.z > speed):
		bebop1_control_cmd.linear.z = speed
	if(bebop1_control_cmd.linear.x < -speed):
		bebop1_control_cmd.linear.x = -speed
	if(bebop1_control_cmd.linear.y < -speed):
		bebop1_control_cmd.linear.y = -speed
	if(bebop1_control_cmd.linear.z < -speed):
		bebop1_control_cmd.linear.z = -speed
		
	print('Cmd Vel\nLinear\nX: ', bebop1_control_cmd.linear.x, ' Y: ', bebop1_control_cmd.linear.y, ' Z: ', bebop1_control_cmd.linear.z)
	
def Update():
	getPosError()
	getVel()
	getAbstError()
	setCmdVel()
	bebop1_cmd_pub.publish(bebop1_control_cmd)
	bebop1.prevx = bebop1.posx
	bebop1.prevy = bebop1.posy
	bebop1.prevz = bebop1.posz

	
	
def Hover():
	bebop1_control_cmd.linear.x = 0.0
	bebop1_control_cmd.linear.y = 0.0
	bebop1_control_cmd.linear.z = 0.0
	bebop1_control_cmd.angular.z = 0.0
	
def getJoy(joy_state):
	global button1 
	button1 = joy_state.buttons[0]
	global button2 
	button2 = joy_state.buttons[1]
	global button3
	button3 = joy_state.buttons[2]
	global button4
	button4 = joy_state.buttons[3]
	

def truncate(number, digits):
    stepper = pow(10.0, digits)
    return math.trunc(stepper * number) / stepper


rospy.init_node('BebopCon', anonymous=True)

takeoff = Empty()
land = Empty()
emergency = Empty()
takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 10)
land_pub = rospy.Publisher('/bebop/land', Empty, queue_size = 10)
emergency_pub = rospy.Publisher('/bebop/reset', Empty, queue_size = 10)
bebop1_cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=0)
rate = rospy.Rate(250)

rospy.Subscriber('/joy', Joy, getJoy)
rospy.Subscriber('vrpn_client_node/Bebop1/pose', PoseStamped, getPose)

start_time = time.time()

setGoal(0.0,0.0,0.5)

while not rospy.is_shutdown():
	elapsedTime = time.time() - start_time

	if(elapsedTime > 2):
		while(operating == 1):
			
			Update()
			
			if(button2==1):
				land_pub.publish(land)
				print('\nLAND\n')
				operating = 0
			if(button3==1):
				emergency_pub.publish(emergency)
				print('RESET')
				quit()
			rate.sleep()
		if(button1==1):
			takeoff_pub.publish(takeoff)
			print('\nTAKEOFF\n')
			operating = 1