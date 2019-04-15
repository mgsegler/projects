#!/usr/bin/python
from __future__ import print_function
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

percentError = 5.0

kp_xy = 0
kd_xy = 0
ki_xy = 0

kp_z = 0.5
kd_z = 0
ki_z = 0

kp_yaw = 0.1
kd_yaw = 1
ki_yaw = 0

speed = 0.5
rate = 100
T = 1/rate

bias = 90

rotationController = False

class Bebop:
	def __init__(self):

		self.pos_x = 0.0
		self.pos_y = 0.0
		self.pos_z = 0.0

		self.ori_x = 0.0
		self.ori_y = 0.0
		self.ori_z = 0.0
		self.ori_w = 0.0

		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0
		self.yaw_rad = 0.0

		self.vel_x = 0.0
		self.vel_y = 0.0
		self.vel_z = 0.0
		self.vel_yaw = 0.0

		self.prev_x = 0.0
		self.prev_y = 0.0
		self.prev_z = 0.0
		self.prev_yaw = 0.0

		self.goal_pos_x = 0.0
		self.goal_pos_y = 0.0
		self.goal_pos_z = 0.0
		self.goal_yaw = 0.0

		self.error_pos_x = 0.0
		self.error_pos_y = 0.0
		self.error_pos_z = 0.0
		self.error_pos_yaw = 0.0
		self.error_prev_pos_x = 0.0
		self.error_prev_pos_y = 0.0
		self.error_prev_pos_z = 0.0
		self.error_prev_pos_yaw = 0.0

		self.error_derv_x = 0.0
		self.error_derv_y = 0.0
		self.error_derv_z = 0.0
		self.error_derv_yaw = 0.0

		self.error_int_x = 0.0
		self.error_int_y = 0.0
		self.error_int_z = 0.0
		self.error_int_yaw = 0.0

		self.error_percent_x = 0.0
		self.error_percent_y = 0.0
		self.error_percent_z = 0.0
		self.error_percent_yaw = 0.0

		self.control_cmd = Twist()
		self.control_cmd.linear.x = 0.0
		self.control_cmd.linear.y = 0.0
		self.control_cmd.linear.z = 0.0
		self.control_cmd.angular.x = 0.0
		self.control_cmd.angular.y = 0.0
		self.control_cmd.angular.z = 0.0

	def getPose(self, data):
		self.pos_x = self.truncate(data.pose.position.x, 3)
		self.pos_y = self.truncate(data.pose.position.z, 3)	#Optitrack uses XZY coordinates
		self.pos_z = self.truncate(data.pose.position.y, 3)
		self.ori_x = self.truncate(data.pose.orientation.x, 3)
		self.ori_y = self.truncate(data.pose.orientation.z, 3)
		self.ori_z = self.truncate(data.pose.orientation.y, 3)
		self.ori_w = self.truncate(data.pose.orientation.w, 3)

		self.bebop_quat = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
		(self.roll, self.pitch, self.yaw_rad) = euler_from_quaternion(self.bebop_quat)

		self.yaw = self.truncate(math.degrees(self.yaw_rad)+bias, 0)

	def setGoal(self, goal_x, goal_y, goal_z, goal_yaw):
		self.goal_pos_x = goal_x
		self.goal_pos_y = goal_y
		self.goal_pos_z = goal_z
		self.goal_yaw = goal_yaw

	def getPosError(self):
		self.error_pos_x = self.goal_pos_x - self.pos_x
		self.error_pos_y = self.goal_pos_y - self.pos_y
		self.error_pos_z = self.goal_pos_z - self.pos_z
		self.error_pos_yaw = self.goal_yaw - self.yaw

		self.prev_x = self.pos_x
		self.prev_y = self.pos_y
		self.prev_z = self.pos_z
		self.prev_yaw = self.yaw

	def getDervError(self):
		self.error_derv_x = self.error_pos_x - self.error_prev_pos_x
		self.error_derv_y = self.error_pos_y - self.error_prev_pos_y
		self.error_derv_z = self.error_pos_z - self.error_prev_pos_z
		self.error_derv_yaw = self.error_pos_yaw - self.error_prev_pos_yaw

		self.error_prev_pos_x = self.error_pos_x
		self.error_prev_pos_y = self.error_pos_y
		self.error_prev_pos_z = self.error_pos_z
		self.error_prev_pos_yaw = self.error_pos_yaw

	def getVel(self):
		self.vel_x = (self.prev_x - self.pos_x)/T
		self.vel_y = (self.prev_y - self.pos_y)/T
		self.vel_z = (self.prev_z - self.pos_z)/T
		self.vel_yaw = (self.prev_yaw - self.yaw)/T

	def getIntError(self):
		self.error_int_x += self.error_pos_x
		self.error_int_y += self.error_pos_y
		self.error_int_z += self.error_pos_z
		self.error_int_yaw += self.error_pos_yaw

		self.error_int_x = self.truncate(self.error_int_x, 3)
		self.error_int_y = self.truncate(self.error_int_y, 3)
		self.error_int_z = self.truncate(self.error_int_z, 3)
		self.error_int_yaw = self.truncate(self.error_int_yaw, 0)

	def setCmdVel(self, x=None, y=None, z=None, yaw=None):
		if (x and y and z and yaw) is not None:
			self.control_cmd.linear.x = x
			self.control_cmd.linear.y = y
			self.control_cmd.linear.z = z
			self.control_cmd.angular.z = yaw

		else:
			self.control_cmd.linear.x = self.truncate(((kp_xy*self.error_pos_x) + (kd_xy*self.error_derv_x) + (ki_xy*self.error_int_x))*math.cos(math.radians(self.yaw)) + -((kp_xy*self.error_pos_y) + (kd_xy*self.error_derv_y) + (ki_xy*self.error_int_y))*math.sin(math.radians(self.yaw)), 2)
			self.control_cmd.linear.y = self.truncate(((kp_xy*self.error_pos_x) + (kd_xy*self.error_derv_x) + (ki_xy*self.error_int_x))*math.sin(math.radians(self.yaw)) + -((kp_xy*self.error_pos_y) + (kd_xy*self.error_derv_y) + (ki_xy*self.error_int_y))*math.cos(math.radians(self.yaw)), 2)
			self.control_cmd.linear.z = self.truncate((kp_z*self.error_pos_z) + (kd_z*self.error_derv_z) + (ki_z*self.error_int_z), 2)
			if rotationController == True:
				self.control_cmd.angular.z = self.truncate((kp_yaw*self.error_pos_yaw) + (kd_yaw*self.error_derv_yaw) + (ki_yaw*self.error_int_yaw), 1)
			else:
				self.control_cmd.angular.z = 0

			if self.control_cmd.linear.x > speed:
				self.control_cmd.linear.x = speed
			elif self.control_cmd.linear.x < -speed:
				self.control_cmd.linear.x = -speed
			if self.control_cmd.linear.y > speed:
				self.control_cmd.linear.y = speed
			elif self.control_cmd.linear.y < -speed:
				self.control_cmd.linear.y = -speed
			if self.control_cmd.linear.z > speed:
				self.control_cmd.linear.z = speed
			elif self.control_cmd.linear.z < -speed:
				self.control_cmd.linear.z = -speed
			if self.control_cmd.angular.z > speed:
				self.control_cmd.angular.z = speed
			elif self.control_cmd.angular.z < -speed:
				self.control_cmd.angular.z = -speed

	def Update(self):
		self.getPosError()
		self.getDervError()
		self.getIntError()
		self.setCmdVel()
		self.goalCheck()

		print('\nCurrent Position:\n\t', 'X: ', self.pos_x, ' Y: ', self.pos_y, ' Z: ', self.pos_z)
		print('\tOrientation:\n\t', 'X: ', self.ori_x, ' Y: ', self.ori_y, ' Z: ', self.ori_z, ' W: ', self.ori_w, ' Yaw (deg): ', self.yaw)
		print('Goal\n\t', 'X: ', self.goal_pos_x, ' Y: ', self.goal_pos_y, ' Z: ', self.goal_pos_z,' Yaw: ', self.goal_yaw)
		print('P Error\n\t', 'X: ', self.error_pos_x, ' Y: ', self.error_pos_y, ' Z: ', self.error_pos_z, ' Yaw: ', self.error_pos_yaw)
		print('I Error\n\t', 'X: ', self.error_int_x, ' Y: ', self.error_int_y, ' Z: ', self.error_int_z, ' Yaw: ', self.error_int_yaw)
		print('D Error\n\t', 'X: ', self.error_derv_x, ' Y: ', self.error_derv_y, ' Z: ', self.error_derv_z, ' Yaw: ', self.error_derv_yaw)
		print('Command Velocity\n\t', 'X: ', self.control_cmd.linear.x, ' Y: ', self.control_cmd.linear.y, ' Z: ', self.control_cmd.linear.z, ' Yaw: ', self.control_cmd.angular.z)

	def Hover(self):
		self.control_cmd.linear.x = 0
		self.control_cmd.linear.y = 0
		self.control_cmd.linear.z = 0
		self.control_cmd.angular.z = 0

	def goalCheck(self):
		self.error_percent_x = ((self.goal_pos_x - self.goal_pos_x)/self.goal_yaw)*100)
		self.error_percent_y = ((self.goal_pos_y - self.goal_pos_y)/self.goal_yaw)*100)
		self.error_percent_z = ((self.goal_pos_z - self.goal_pos_z)/self.goal_yaw)*100)
		self.error_percent_yaw = ((self.goal_pos_yaw - self.goal_pos_yaw)/self.goal_yaw)*100)

		if self.error_percent_x < percentError:
			self.control_cmd.linear.x = 0
		if self.error_percent_y < percentError:
			self.control_cmd.linear.y = 0
		if self.error_percent_z < percentError:
			self.control_cmd.linear.z = 0
		if self.error_percent_yaw < percentError:
			self.control_cmd.angular.z = 0

	def truncate(self, number, digits):
	    stepper = pow(10.0, digits)
	    return math.trunc(stepper * number) / stepper
