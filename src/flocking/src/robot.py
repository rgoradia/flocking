from geometry_msgs.msg import Twist
import numpy as np
import math
import rospy
import time
from math import sin, cos, asin, acos, atan2, sqrt
from matplotlib import pyplot as plt



class Robot():
	def __init__(self,name, angle_sign=0, reference_vel=0.0):
		self.first_time = True
		self.turtlebot_pos_start_frame = [0,0,0]
		self.turtlebot_vel = 0
		self.turtlebot_w = 0
		self.start_theta = 0
		self.g_sw = []
		self.reference_vel = reference_vel
		self.e_p = 0
		self.e_w = 0
		self.last_e_v = 0
		self.last_e_w = 0
		self.g_laser_curr_bot = []
		#Obstacle avoidance variables below
		self.avoiding_obstacle = False
		self.avoiding_start_time = -1
		self.avoiding_start_theta = 0
		self.angle_sign = angle_sign

	def update_odometry(self,x,y,theta,vel=0,w=0):
		if self.first_time:
			self.g_sw = np.linalg.inv(self.rigid(np.array([x,y,theta])))
			self.start_theta = theta
			self.turtlebot_vel = vel
			self.turtlebot_w = w
			self.first_time = False
		else:
			[x,y,idk] = np.dot(self.g_sw,np.array([x,y,1]))
			self.turtlebot_pos_start_frame[0] = x
			self.turtlebot_pos_start_frame[1] = y
			self.turtlebot_pos_start_frame[2] = theta - self.start_theta
			self.turtlebot_w = w
			self.turtlebot_vel = vel


	def rigid(self, twist):
		return np.array([
				[cos(twist[2]), -sin(twist[2]), twist[0]],
				[sin(twist[2]), cos(twist[2]), twist[1]],
				[0, 0, 1]
			])

	def get_current_pos_inital_frame(self):
		return self.turtlebot_pos_start_frame

	def get_desired_turtle_pos_turtle_frame(self, des_pos, curr_rad, curr_angle, ridgeback_angle):
	    curr_xy = np.array([curr_rad * np.cos(curr_angle), curr_rad * np.sin(curr_angle)])
	    diff_theta =0
	    self.g_laser_curr_bot = np.array([[np.cos(diff_theta), -np.sin(diff_theta), curr_xy[0]],
	        [np.sin(diff_theta), np.cos(diff_theta), curr_xy[1]],
	        [0, 0, 1]])
	    des_pos_t_frame = np.dot(np.linalg.inv(self.g_laser_curr_bot), np.array([des_pos[0], des_pos[1], 1]))
	    des_pos_t_frame[2] = ridgeback_angle - self.get_current_pos_inital_frame()[2]
	    return des_pos_t_frame

	def check_if_near_obstacle(self, obstacle):
		t_frame_obj = np.dot(np.linalg.inv(self.g_laser_curr_bot), np.array([obstacle[0], obstacle[1], 1]))
		if np.linalg.norm(t_frame_obj[:2]) < 0.2:
			self.avoiding_obstacle = True

	def control_bot(self,target,del_t, obstacle):
		# if self.avoiding_obstacle:
		# 	curr_t = rospy.Time.from_sec(time.time()).to_sec()
		# 	vel_msg = Twist()
		# 	if self.avoiding_start_time == -1:
		# 		self.avoiding_start_time = curr_t
		# 		self.avoiding_start_theta = self.get_current_pos_inital_frame()[2]

		# 	diff_t = curr_t - self.avoiding_start_time
		# 	diff_theta = np.absolute(self.get_current_pos_inital_frame()[2] - self.avoiding_start_theta)
		# 	if diff_t < 0.75 and np.absolute(np.pi / 2 - diff_theta) > 0.1:
		# 		vel_msg.angular.z = self.angle_sign * 2
		# 	elif diff_t < 2.0:
		# 		vel_msg.linear.x = 0.3
		# 	elif diff_t < 2.6 and diff_theta > 0.1:
		# 		vel_msg.angular.z = -self.angle_sign * 2
		# 	elif diff_t < 4.0:
		# 		vel_msg.linear.x = 0.3
		# 	elif diff_t < 4.75 and np.absolute(np.pi / 2 - diff_theta) > 0.1:
		# 		vel_msg.angular.z = -self.angle_sign * 2
		# 	elif diff_t < 5.0:
		# 		vel_msg.linear.x = 0.3
		# 	elif diff_t < 5.75 and diff_theta > 0.1:
		# 		vel_msg.angular.z = self.angle_sign * 2
		# 	else:
		# 		self.avoiding_start_time = -1
		# 		self.avoiding_obstacle = False
		# 	return vel_msg

		#print(obstacle)

		if obstacle:
			#print(obstacle[0] - target[0])
			#print(np.arccos((obstacle[0] - target[0]) / max(0.5, np.absolute(obstacle[0] - target[0]))))
			target[1] = target[1] + self.angle_sign * np.sqrt(max(0.5, np.absolute(obstacle[0] - target[0]))**2 - (obstacle[0] - target[0])**2)
			target[2] = target[2] + (self.angle_sign * np.arcsin((obstacle[0] - target[0]) / max(0.5, np.absolute(obstacle[0] - target[0]))) - self.get_current_pos_inital_frame()[2])
			#print(target)
			#print('****')

		target_x = target[0]
		target_y = target[1]
		target_theta = target[2]

		curr_x = self.turtlebot_pos_start_frame[0]
		curr_y = self.turtlebot_pos_start_frame[1]
		curr_theta = self.turtlebot_pos_start_frame[2]

		#g_tc_ts = np.linalg.inv(self.rigid(self.turtlebot_pos_start_frame))
		#target = np.dot(g_tc_ts, np.array([target_x, target_y, 1]))
		#target[2] = target_theta - curr_theta

		target_x = target[0]
		target_y = target[1]
		target_theta = target[2]
		curr_x = 0
		curr_y = 0
		curr_theta = 0

		k1 = 5#1.5
		k2 = 5#5
		k3 = 3#5
		Kp1 = 0.8
		Kp2 = 0.5
		Ki1 = 0.8
		Ki2 = 0.5

		theta_diff = target_theta - curr_theta
		C = np.diag([cos(theta_diff), 1])
		diff_in_x = target_x - curr_x
		diff_in_y = target_y - curr_y

		Vw = np.array([self.reference_vel, 0.0]).T

		u1 = -k1 * diff_in_x
		u2 = k2*np.abs(self.reference_vel)*np.sinc(theta_diff)*(diff_in_y) - k3*(theta_diff)
		U = np.array([u1, u2]).T
		Vw_d = np.dot(C, Vw) - U

		e_v = self.turtlebot_vel - Vw_d[0]
		self.e_p = e_v*del_t + 0.9*self.e_p
		self.last_e_v = e_v

		e_w = self.turtlebot_w - Vw_d[1]
		self.e_w = e_w*del_t + 0.9*self.e_w
		self.last_e_w = e_w

		e_am = -Kp1*e_v - Ki1*self.e_p
		e_ad = -Kp2*e_w - Ki2*self.e_w

		#print '****'
		#print e_am
		#print e_ad
		#print '####'

		vel_msg = Twist()
		vel_msg.linear.x = e_am
		vel_msg.angular.z = e_ad	
		return vel_msg
