#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from flocking.msg import Polar

turtle_r_pub = rospy.Publisher('turtle_r_pos', Polar, queue_size=10)
turtle_l_pub = rospy.Publisher('turtle_l_pos', Polar, queue_size=10)

def find_turtlebot():
    rospy.init_node('find_turtle')
    angles = np.linspace(-2.35619449615, 2.35619449615, 1080)
    step = angles[1] - angles[0]
    while not rospy.is_shutdown():
        lidar_sub = rospy.Subscriber('/front/scan', LaserScan, laserCallback, (angles, step))

def laserCallback(msg, args):
    angles = args[0]
    step = args[1]
    zipped = zip(msg.ranges, angles)
    valid_range, valid_angle = zip(*[x for x in zipped if x[0] < np.sqrt(2)])

    angle_diff = np.ones(len(valid_angle)) * step
    angle_diff[0] = valid_angle[0]
    angle_diff = np.array(list(valid_angle)) - np.cumsum(angle_diff)
    angle_split = np.where(angle_diff < 2e-2)[0][-1]
    angle_split = np.where(np.array(valid_angle) < 0)[0][-1]
    f_a, s_a = np.split(valid_angle, [angle_split])
    f_r, s_r = np.split(valid_range, [angle_split])
    splits = [[f_a, f_r], [s_a,s_r]]

    if len(splits) < 2:
        print('Less than 2 turtlebots found')
        return
    turtle_r = splits[0]
    turtle_l = splits[1]
    if len(splits) > 2:
        split_length = [len(i[0]) for i in splits]
        s_ind = sorted(range(len(split_length)), key=lambda x:split_length[x])
        s1 = s_ind[-1]
        s2 = s_ind[-2]
        turtle_r = splits[min(s1,s2)]
        turtle_l = splits[max(s2,s1)]

    turtle_r = [np.mean(turtle_r[0]), np.mean(turtle_r[1])]
    turtle_l = [np.mean(turtle_l[0]), np.mean(turtle_l[1])]

    turtle_r_pub.publish(Polar(turtle_r[0], turtle_r[1]))
    turtle_l_pub.publish(Polar(turtle_l[0], turtle_l[1]))

if __name__ == '__main__':
    find_turtlebot()
