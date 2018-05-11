#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from flocking.msg import TurtlePos
#T2 = left
MAX_RAD = np.sqrt(2)

turtle_pos = rospy.Publisher('init_turtle_pos', TurtlePos, queue_size=10)

def find_turtlebot(angles, step):
    lidar_sub = rospy.Subscriber('/front/scan', LaserScan, laserCallback, (angles, step))

def laserCallback(msg, args):
    angles = args[0]
    step = args[1]
    zipped = zip(msg.ranges, angles)
    valid_range, valid_angle = zip(*[x for x in zipped if x[0] < MAX_RAD])

    angle_diff = np.ones(len(valid_angle)) * step
    angle_diff[0] = valid_angle[0]
    angle_diff = np.array(list(valid_angle)) - np.cumsum(angle_diff)
    #range_diff = np.zeros(len(valid_range))
    #range_diff[1:] = np.diff(valid_range)
    angle_split = np.where(angle_diff < 3e-2)[0][-1] + 1
    #range_split = np.where(range_diff < 0.1)[0][-1]
    #f_a, s_a = np.split(valid_angle, [min(angle_split, range_split)])
    #f_r, s_r = np.split(valid_range, [min(angle_split, range_split)])
    f_a, s_a = np.split(valid_angle, [angle_split])
    f_r, s_r = np.split(valid_range, [angle_split])
    pos_splits = [[f_a, f_r]]
    neg_splits = []
    while s_a.size:
        #split_i = min(angle_split, range_split) + 1
        angle_diff = angle_diff[angle_split:]
        #print(valid_angle[angle_split:])
        angle_split = np.where(angle_diff < angle_diff[0] + 3e-2)[0][-1] + 1
        #range_split = np.where(range_diff < range_diff[min(split_i, len(range_diff)-1)] + 0.1)[0][-1]
        #if angle_split < range_split:
        #    print('angle: ', angle_split)
        #    print(range_split)
        #else:
        #    print('range: ', range_split)
        #    print(angle_split)
        #f_a, s_a = np.split(s_a, [min(angle_split, range_split)])
        #f_r, s_r = np.split(s_r, [min(angle_split, range_split)])
        f_a, s_a = np.split(s_a, [angle_split])
        f_r, s_r = np.split(s_r, [angle_split])
        #print(f_a)
        #print(s_a)
        if f_a[0] < 0:
            pos_splits.append([f_a, f_r])
        else:
            neg_splits.append([f_a,f_r])

    if pos_splits == [] or neg_splits == []:
        print('Fewer than 2 turtlebots found')
        return
    turtle_r = pos_splits[0]
    turtle_l = neg_splits[0]
    if len(pos_splits) > 1:
        split_length = [len(i[0]) for i in pos_splits]
        s_ind = sorted(range(len(split_length)), key=lambda x:split_length[x])
        s1 = s_ind[-1]
        turtle_r = pos_splits[s1]

    if len(neg_splits) > 1:
        split_length = [len(i[0]) for i in neg_splits]
        s_ind = sorted(range(len(split_length)), key=lambda x:split_length[x])
        s1 = s_ind[-1]
        turtle_l = neg_splits[s1]

    turtle_r = [np.mean(turtle_r[0]), np.mean(turtle_r[1])]
    turtle_l = [np.mean(turtle_l[0]), np.mean(turtle_l[1])]

    turtle_pos.publish(TurtlePos(turtle_r[0], turtle_r[1], turtle_l[0], turtle_l[1]))

if __name__ == '__main__':
    rospy.init_node('find_turtle')
    angles = np.linspace(-2.35619449615, 2.35619449615, 1081)
    step = angles[1] - angles[0]
    while not rospy.is_shutdown():
        find_turtlebot(angles, step)
        rospy.spin()
