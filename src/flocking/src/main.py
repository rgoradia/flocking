#!/usr/bin/env python
"""
Starter script for final project. 
Author: Rushil Goradia/Sachiko Matsumoto
"""
import copy
import rospy
import sys
import argparse

import tf
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from flocking.msg import TurtlePos, ObjectPos
from robot import *

TURTLE_RAD = 0.1

turtlebot_left = 'blue/'
turtlebot_right = 'red/'
obstacle_left_path = False
obstacle_right_path = False
#current_path = LinearPath(5)
#current_path = arc
left_bot = Robot(turtlebot_left, 1)
right_bot = Robot(turtlebot_right, -1)
ridgeback = Robot('ridgeback')

cmd_vel_center = rospy.Publisher('ridgeback_velocity_controller/cmd_vel', Twist, queue_size=10)
cmd_vel_left = rospy.Publisher(turtlebot_left+'cmd_vel_mux/input/teleop', Twist, queue_size=10)
cmd_vel_right = rospy.Publisher(turtlebot_right+'cmd_vel_mux/input/teleop', Twist, queue_size=10)

#listener = tf.TransformListener()

def main():
    rospy.init_node('flocking', anonymous=False)
    rospy.sleep(1.0)

    rospy.loginfo("To stop Robots CTRL + C")
    rospy.on_shutdown(shutdown)
    rospy.sleep(1.0)

    rate = rospy.Rate(10)

    global right_pos, right_angle, left_pos, left_angle, last_right_xy, last_left_xy, right_obstacle, left_obstacle
    right_pos = 0
    right_angle = 0
    left_pos = 0
    left_angle = 0
    last_right_xy = 0
    last_left_xy = 0
    right_obstacle = []
    left_obstacle = []

    center_vel = rospy.Subscriber('ridgeback_velocity_controller/odom', Odometry, odomCallbackCenter)
    left_vel = rospy.Subscriber(turtlebot_left+'odom', Odometry, odomCallbackLeft)
    right_vel = rospy.Subscriber(turtlebot_right+'odom', Odometry, odomCallbackRight)
    turtlebot_tracker = rospy.Subscriber('turtle_pos', TurtlePos, turtlebotTrackerCallback)
    obstacle_positions = rospy.Subscriber('object_pos', ObjectPos, obstacleCallback)
    lidar_position = []
    lidar_angle = []
    rospy.sleep(1.0)

    
    right_init_r = right_pos
    right_init_theta = right_angle
    right_init_xy = np.array([(right_init_r) * np.cos(right_init_theta), (right_init_r) * np.sin(right_init_theta)])
    # right_bot_g_st_sr = np.dot(np.array([[1, 0, right_init_xy[0]],
    #     [0, 1, right_init_xy[1]],
    #     [0, 0, 1]]), np.linalg.inv(g_ridgebackbase_laser))
    left_init_r = left_pos
    left_init_theta = left_angle
    left_init_xy = np.array([left_init_r * np.cos(left_init_theta), left_init_r * np.sin(left_init_theta)])
    #left_bot_g_st_sr = np.dot(np.array([[1, 0, left_init_xy[0]],
    #    [0, 1, left_init_xy[1]],
    #    [0, 0, 1]]), np.linalg.inv(g_ridgebackbase_laser))

    right_err = []
    left_err = []
    time_err = []

    #init_r = 0.7
    #init_theta = np.pi/4
    #left_bot_g_st_sr = np.linalg.inv(rigid(np.array([init_r*np.cos(init_theta), init_r*np.sin(init_theta), 0])))

    
    prev_time = rospy.Time.from_sec(time.time()).to_sec()
    while not rospy.is_shutdown():

        #Change to only ridgeback
        straightLine()
        ridgeback_curr_pos_sr_frame = ridgeback.get_current_pos_inital_frame()
        #left_bot_target_pos_st_frame = calculate_turtlebot_target_pos_st(ridgeback.get_current_pos_inital_frame(), init_r, init_theta, left_bot_g_st_sr)
        #print left_bot_target_pos_st_frame


        #right_err.append(np.linalg.norm(last_right_xy - right_init_xy))
        #left_err.append(np.linalg.norm(last_left_xy - left_init_xy))
        #time_err.append(prev_time)
        
        right_bot_target_pos_rt_frame = right_bot.get_desired_turtle_pos_turtle_frame(right_init_xy, right_pos, right_angle, ridgeback_curr_pos_sr_frame[2])
        left_bot_target_pos_lt_frame = left_bot.get_desired_turtle_pos_turtle_frame(left_init_xy, left_pos, left_angle, ridgeback_curr_pos_sr_frame[2])
        #right_bot_target_pos_rt_frame[2] = ridgeback_curr_pos_sr_frame[2] - right_bot.get_current_pos_inital_frame()[2]
        #left_bot_target_pos_lt_frame[2] = ridgeback_curr_pos_sr_frame[2] - left_bot.get_current_pos_inital_frame()[2]

        del_t = rospy.Time.from_sec(time.time()).to_sec() - prev_time
        prev_time = prev_time + del_t
        #left_vel = left_bot.control_bot(left_bot_target_pos_st_frame,del_t)
        #right_vel = right_bot.control_bot(ridgeback.get_current_pos_inital_frame(),del_t)
        #right_bot.avoiding_obstacle = True
        right_obstacle = []
        right_vel = right_bot.control_bot(right_bot_target_pos_rt_frame, del_t, right_obstacle)
        left_vel = left_bot.control_bot(left_bot_target_pos_lt_frame, del_t, left_obstacle)


        cmd_vel_left.publish(left_vel)
        cmd_vel_right.publish(right_vel)
        #lidar_position += [pos] 
        #lidar_angle += [angle]
        rate.sleep()

    #plt.figure(1)
    #colors = ['blue', 'green', 'red']
    #labels = ['x', 'y']
    #plt.subplot(211)
    #plt.plot(-np.array(lidar_position)*np.sin(lidar_angle), color=colors[0], ls='solid', label=labels[0])
    #plt.plot(lidar_position*np.cos(lidar_angle), color=colors[1], ls='dotted',label=labels[1])
    #plt.legend()
    
    # plt.figure()
    # colors = ['blue', 'red']
    # labels = ['right error', 'left error']
    # plt.plot([time_err[i] - time_err[0] for i in range(len(time_err))], right_err, color=colors[0], ls='solid', label=labels[0])
    # plt.plot([time_err[i] - time_err[0] for i in range(len(time_err))], left_err, color=colors[1], ls='solid', label=labels[1])
    # plt.xlabel('Time (s)')
    # plt.ylabel('l_2 error (m)')
    # plt.title('Turtlebot position error during a linear path')
    # plt.legend()
    
    # colors = ['blue', 'green', 'red']
    # labels = ['r', 'theta']
    # plt.subplot(212)
    # plt.plot(lidar_position, color=colors[0], ls='solid', label=labels[0])
    # plt.plot(lidar_angle, color=colors[1], ls='dotted',label=labels[1])
    # plt.legend()
    # plt.show()

# This method gives you position AND velocity if you need it
def odomCallbackCenter(msg):
    x = msg.pose.pose.position.x + 0.393
    y = msg.pose.pose.position.y
    theta = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    vel = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
    w = msg.twist.twist.angular.z
    ridgeback.update_odometry(x,y,theta,vel,w)

def odomCallbackLeft(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    vel = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
    w = msg.twist.twist.angular.z
    left_bot.update_odometry(x,y,theta,vel,w)

def odomCallbackRight(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    vel = (msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)**0.5
    w = msg.twist.twist.angular.z
    right_bot.update_odometry(x,y,theta,vel,w)

def turtlebotTrackerCallback(msg):
    global right_pos, right_angle, left_pos, left_angle, last_right_xy, last_left_xy
    last_right_xy = [(right_pos) * np.cos(right_angle), (right_pos) * np.sin(right_angle)]
    last_left_xy = [(left_pos ) * np.cos(left_angle), (left_pos) * np.sin(left_angle)]
    right_pos = msg.t1_radius
    right_angle = msg.t1_angle
    left_pos = msg.t2_radius
    left_angle = msg.t2_angle

def obstacleCallback(msg):
    global right_obstacle, left_obstacle
    if msg.angle[0] == 0:
        right_obstacle = []
        left_obstacle = []
        return
    if msg.angle[0] > 0:
        right_obj_angle, left_obj_angle = [], msg.angle
        right_obj_dist, left_obj_dist = [], msg.distance
        right_obj_radii, left_obj_radii = [], msg.radius
        right_obstacle = []
    elif msg.angle[-1] < 0:
        right_obj_angle, left_obj_angle = msg.angle, []
        right_obj_dist, left_obj_dist = msg.distance, []
        right_obj_radii, left_obj_radii = msg.radius, []
        left_obstacle = []
    else:
        split = np.where(np.array(msg.angle) < 0)[0][0] + 1
        right_obj_angle, left_obj_angle = np.split(msg.angle, [split])
        right_obj_dist, left_obj_dist = np.split(msg.distance, [split])
        right_obj_radii, left_obj_radii = np.split(msg.radius, [split])
    right_obj_xy = [[right_obj_dist[i] * np.cos(right_obj_angle[i]), right_obj_dist[i] * np.sin(right_obj_angle[i])] for i in range(len(right_obj_angle))]
    left_obj_xy = [[left_obj_dist[i] * np.cos(left_obj_angle[i]), left_obj_dist[i] * np.sin(left_obj_angle[i])] for i in range(len(left_obj_angle))]
    right_obj_xy = sorted(right_obj_xy, key=lambda x: x[0])
    left_obj_xy = sorted(left_obj_xy, key=lambda x: x[0])
    right_bot_xy = np.array([right_pos * np.cos(right_angle), right_pos * np.sin(right_angle)])
    left_bot_xy = np.array([left_pos * np.cos(left_angle), left_pos * np.sin(left_angle)])
    for i in range(len(right_obj_xy)):
        if np.absolute(right_obj_xy[i][0] - right_bot_xy[0]) < 0.5 and np.absolute(right_bot_xy[1] - right_obj_xy[i][1]) < right_obj_radii[i] + TURTLE_RAD:
            #obstacle_detected_right_path(right_obj_xy[i], right_obj_radii[i], right_bot_xy)
            right_bot.avoiding_obstacle = True
            right_obstacle = right_obj_xy[i]
            break
    for i in range(len(left_obj_xy)):
        if np.absolute(left_obj_xy[i][0] - left_bot_xy[0]) < 0.5 and np.absolute(left_bot_xy[1] - left_obj_xy[i][1]) < left_obj_radii[i] + TURTLE_RAD:
            #obstacle_detected_left_path(left_obj_xy[i], left_obj_radii[i], left_bot_xy)
            left_bot.avoiding_obstacle = True
            left_obstacle = left_obj_xy[i]
            break

def calculate_turtlebot_target_pos_st(ridgeback_pos, init_r, init_theta, g_st_sr):
    ridgeback_x = ridgeback_pos[0]
    ridgeback_y = ridgeback_pos[1]
    ridgeback_theta = ridgeback_pos[2]

    turtle_x = ridgeback_x + init_r*np.cos(ridgeback_theta+init_theta)
    turtle_y = ridgeback_y + init_r*np.sin(ridgeback_theta+init_theta)

    left_bot_target_pos_st_frame = np.dot(g_st_sr, np.array([turtle_x, turtle_y,1]))
    left_bot_target_pos_st_frame[2] = ridgeback_theta

    return left_bot_target_pos_st_frame

def rigid(twist):
        return np.array([
                [cos(twist[2]), -sin(twist[2]), twist[0]],
                [sin(twist[2]), cos(twist[2]), twist[1]],
                [0, 0, 1]
            ])

def straightLine():
    vel = Twist()
    vel.linear.x = 0.1
    #cmd_vel_center.publish(vel)
    #cmd_vel_left.publish(vel)
    #cmd_vel_right.publish(vel)

def shutdown():
    rospy.loginfo("Stopping Robots")
    cmd_vel_center.publish(Twist())
    cmd_vel_left.publish(Twist())
    cmd_vel_right.publish(Twist())
    rospy.sleep(1)

def obstacle_detected_left_path(obstacle_pos, obstacle_radius, turtlebot_pos):
    obstacle_left_path = True
    go_around_obstacle(turtlebot_left)
    obstacle_left_path = False

def obstacle_detected_right_path(obstacle_pos, obstacle_radius, turtlebot_pos):
    obstacle_right_path = True
    go_around_obstacle(turtlebot_right)
    obstacle_right_path = False

def get_2d_tf_transform(from_frame, to_frame):
    global listener
    rospy.sleep(1.0)
    if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
        print 'Frames not found'
        exit(0)
    t = listener.getLatestCommonTime(from_frame, to_frame)
    to_frame_pos, to_frame_rot = listener.lookupTransform(from_frame, to_frame, t)
    theta_x, theta_y, theta_z = euler_from_quaternion(to_frame_rot)
    g = np.array([[np.cos(theta_z), -np.sin(theta_z), to_frame_pos[0]],
        [np.sin(theta_z), np.cos(theta_z), to_frame_pos[1]],
        [0,0,1]])
    return g



if __name__ == "__main__":
    main()
