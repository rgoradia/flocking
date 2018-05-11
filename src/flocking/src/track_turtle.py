#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import quaternion_matrix
from flocking.msg import TurtlePos, ObjectPos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

MAX_TURTLE = 1.0
MAX_OBJECT = np.sqrt(2)

last_frames = np.zeros((2,1080))
last_int = np.zeros(1080)
last_tr = 0
last_tl = 0
last_vr = 0
last_vl = 0
last_t = 0
last_obj = []
lidar_data = 0
angles = np.linspace(-2.35619449615, 2.35619449615, 1081)
step = angles[1] - angles[0]

turtle_pos = rospy.Publisher('turtle_pos', TurtlePos, queue_size=10)
object_pos = rospy.Publisher('object_pos', ObjectPos, queue_size=10)

def find_turtle(angles):
    global last_tr
    while type(lidar_data) is int:
        lidar_sub = rospy.Subscriber('/front/scan', LaserScan, laserCallback)
    #if type(last_tr) is int:
    #    turtle_sub = rospy.Subscriber('/init_turtle_pos', TurtlePos, turtleCallback)
    #turtle_sub.unregister()
    lidar_sub = rospy.Subscriber('/front/scan', LaserScan, laserCallback)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback, (angles))

def segment_lidar(ang, rad):
    ang = ang[np.where(rad < MAX_OBJECT)]
    rad = rad[np.where(rad < MAX_OBJECT)]
    rad_diff = np.zeros(len(rad))
    rad_diff[1:] = np.diff(rad)
    angle_diff = np.ones(len(ang)) * step
    angle_diff[0] = ang[0]
    angle_diff = ang - np.cumsum(angle_diff)
    rad_split = np.where(np.absolute(rad_diff) > 0.05)[0]
    angle_split = np.where(angle_diff < 3e-2)[0][-1] + 1
    if not rad_split.size or angle_split < rad_split[0]:
        split_ind = angle_split
        if not rad_split.size:
            rad_diff = []
    else:
        rad_split = rad_split[0]
        split_ind = rad_split
    f_a, s_a = np.split(ang, [split_ind])
    f_r, s_r = np.split(rad, [split_ind])
    objects = []
    if len(f_a) > 5:
        objects = [[f_a, f_r]]
    while s_a.size:
        if not rad_diff == []:
            rad_diff = rad_diff[split_ind:]
            rad_diff[0] = 0
            rad_split = np.where(np.absolute(rad_diff) > .05)[0]
        angle_diff = angle_diff[split_ind:]
        angle_split = np.where(angle_diff < angle_diff[0] + 3e-2)[0][-1] + 1
        if not rad_split.size or angle_split < rad_split[0]:
            split_ind = angle_split
            if not rad_split.size:
                rad_diff = []
            else:
                rad_split = rad_split[0]
        else:
            rad_split = rad_split[0]
            split_ind = rad_split
        f_a, s_a = np.split(s_a, [split_ind])
        f_r, s_r = np.split(s_r, [split_ind])
        if len(f_a) > 5:
            objects.append([f_a, f_r])
    #print(objects)
    new_objects = [objects[0], objects[1]]
    skip_obj = 0
    skip_obj2 = 1
    for i in range(2, len(objects)):
        obj1 = objects[i - 2]
        obj3 = objects[i]
        if np.absolute(obj3[1][0] - obj1[1][-1]) < .05 and obj1[1][-1] * (obj3[0][0] - obj1[0][-1]) < 0.05:
            new_objects[skip_obj] = np.append(new_objects[skip_obj], obj3, axis=1)
            tmp = skip_obj
            skip_obj = skip_obj2
            skip_obj2 = tmp
        else:
            new_objects.append(obj3)
            skip_obj = skip_obj2
            skip_obj2 = len(new_objects) - 1
    objects = new_objects
    return objects

def laserCallback(msg):
    global lidar_data
    lidar_data = msg

def turtleCallback(msg):
    print('turtle callback')
    global last_tr, last_tl, last_frames, last_int, last_t, lidar_data
    ranges = np.array(lidar_data.ranges)
    intensities = np.array(lidar_data.intensities)
    last_tr = np.array([msg.t1_angle, msg.t1_radius])
    last_tl = np.array([msg.t2_angle, msg.t2_radius])
    print(last_tr, last_tl)
    angles = np.linspace(-2.35619449615, 2.35619449615, 1081) 
    last_frames = np.array([ranges * np.cos(angles), ranges * np.sin(angles)])
    last_int = intensities
    last_t = rospy.get_rostime().nsecs / 1.0e9

def odomCallback(msg, args):
    global last_int, last_tr, last_tl, last_t, lidar_data
    rad = np.array(lidar_data.ranges)
    ang = np.array(args)
    q = msg.pose.pose.orientation
    q = [q.x,q.y,q.z,q.w]
    p = msg.pose.pose.position
    t = rospy.get_rostime().nsecs / 1.0e9
    del_t = t - last_t
    last_t = t
    #g = quaternion_matrix(q)
    #g[0][3] = p.x
    #g[1][3] = p.y
    #g[2][3] = p.z
    #xy_frame = np.array([rad * np.cos(ang), rad * np.sin(ang)])
    #background = xy_frame - last_frames
    #last_frame = xy_frame
    #last_int = args[0].intensities
    objects = segment_lidar(ang, rad)
    if not type(last_tr) is np.ndarray:
        possible_tr = [objects[i] for i in range(len(objects)) if objects[i][1][0] < MAX_TURTLE and objects[i][0][0] < 0]
        possible_tl = [objects[i] for i in range(len(objects)) if objects[i][1][0] < MAX_TURTLE and objects[i][0][0] >= 0]
        split_lengthr = [len(i[0]) for i in possible_tr]
        split_lengthl = [len(i[0]) for i in possible_tl]
        r_ind = sorted(range(len(split_lengthr)), key=lambda x:split_lengthr[x])
        l_ind = sorted(range(len(split_lengthl)), key=lambda x:split_lengthl[x])
        last_tr = np.mean(possible_tr[r_ind[-1]], axis=1)
        last_tl = np.mean(possible_tl[l_ind[-1]], axis=1)
        return
    avg_obj = np.array([np.mean(i,axis=1) for i in objects])
    radii = np.array([[np.sqrt(avg_obj[i][1]**2 + objects[i][1][-1] ** 2 - 2 * avg_obj[i][1] * objects[i][1][-1] * np.cos(avg_obj[i][0] - objects[i][0][-1])) for i in range(len(objects))]])
    avg_obj = np.append(avg_obj, np.transpose(radii), axis=1)

    #norm_tr = last_tr / np.linalg.norm(last_tr)
    #norm_tl = last_tl / np.linalg.norm(last_tl)
    #norm_obj = avg_obj[:,:2] / np.transpose(np.array([np.linalg.norm(avg_obj[:,:2], axis=1)]))
    no_pos_obj = avg_obj[np.where(avg_obj[:,0]<0)][:,:2]
    no_neg_obj = avg_obj[np.where(avg_obj[:,0]>=0)][:,:2]
    if no_pos_obj.size:
        tr_ind = np.argmax(-np.sum(np.absolute(no_pos_obj - last_tr), axis=1))
        tr_ind = np.where(avg_obj[:,:2] == no_pos_obj[tr_ind])[0][0]
        last_tr = avg_obj[tr_ind,:2]
        avg_obj = np.delete(avg_obj, tr_ind, axis=0)
    if no_neg_obj.size:
        tl_ind = np.argmax(-np.sum(np.absolute(no_neg_obj - last_tl), axis=1))
        tl_ind = np.where(avg_obj[:,:2] == no_neg_obj[tl_ind])[0][0]
        last_tl = avg_obj[tl_ind, :2]
        avg_obj = np.delete(avg_obj, tl_ind, axis=0)
    turtle_pos.publish(TurtlePos(last_tr[0], last_tr[1], last_tl[0], last_tl[1]))
    if avg_obj.size:
        object_pos.publish(ObjectPos(avg_obj[:,0], avg_obj[:,1], avg_obj[:,2]))
    else:
        object_pos.publish(ObjectPos([0],[0],[0]))

if __name__ == "__main__":
    rospy.init_node('track_turtle')
    angles = np.linspace(-2.35619449615, 2.35619449615, 1081)
    step = angles[1] - angles[0]
    while not rospy.is_shutdown():
        
        find_turtle(angles)
        rospy.spin()
