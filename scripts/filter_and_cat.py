#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import time
import ros_numpy
import numpy as np
# import pcl

#注册发布
all_point_puber = rospy.Publisher('E_all_points', PointCloud2, queue_size=1)

HEADER = '''\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4 
TYPE F F F F 
COUNT 1 1 1 1 
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii
'''



def cat_callback(high_pc, low_pc):   #数据顺序与filter传入话题顺序一致
    #dajkl
    tic = time.time()
    # points_high = point_cloud2.read_points_list(high_pc)
    points_high = ros_numpy.numpify(high_pc).flatten()
    # print(points_high['x'])
    # print(type(points_high))
    # print(points_high)
    
    points_low = ros_numpy.numpify(low_pc).flatten()
    
    raw = np.zeros((len(points_high)+len(points_low), 4))
    

    raw[:,0] = np.concatenate([points_low['x'],points_high['x']])
    raw[:,1] = np.concatenate([points_low['y'],points_high['y']])
    raw[:,2] = np.concatenate([points_low['z'],points_high['z']])
    raw[:,3] = np.concatenate([points_low['intensity'],points_high['intensity']])

    mask = (raw[:,3] > 0) & \
        (raw[:,0] > -16) & (raw[:,0] < 14) & \
        (raw[:,1] > -27) & (raw[:,1] < 23) & \
        (raw[:,2] < 2) & \
        ((raw[:,0] < 8.2) | (raw[:,1] < 7))      
    raw = raw[mask]
    th = 0.2
    
    mask = ((0.00111842*raw[:,0] + 0.000594686*raw[:,1] + -0.999999*raw[:,2] + -0.0335836) > 0.15) | \
           ((0.00111842*raw[:,0] + 0.000594686*raw[:,1] + -0.999999*raw[:,2] + -0.0335836) < -0.15)
    raw = raw[mask]
    
    print('-------pc process time--------')
    costtime = time.time() - tic
    print(costtime)
    
    
    save_pcd_path = str(low_pc.header.stamp.secs) + ("%f"%(low_pc.header.stamp.nsecs*1e-9))[2:] + ".pcd"
    # cloud = pcl.PointCloud.PointXYZI.from_array(raw)
    n = len(raw)
    lines = []
    for i in range(n):
        x, y, z, i = raw[i]
        lines.append('{:.6f} {:.6f} {:.6f} {}'.format( \
            x, y, z, i))
    with open(save_pcd_path, 'w') as f:
        f.write(HEADER.format(n, n))
        f.write('\n'.join(lines))

    
    # pcl.io.savePCDFileASCII("./f.pcd", cloud)
    
    # points_low = np.reshape(ros_numpy.numpify(low_pc), (-1, 1))
    # print(points_low.shape)
    
    # pc_all = np.stack([points_high,points_low])
    # pc_all = []
    # for p in points_high:
    #     pc_all.append([p[0],p[1],p[2]])
    # for p in points_low:
    #     pc_all.append([p[0],p[1],p[2]])
    
    print('-------save pc time--------')
    costtime = time.time() - tic
    print(costtime)

    # print('----------------high lidar time stamp----------------')
    # print(high_pc.header.stamp.secs)
    # print(high_pc.header.stamp.nsecs)
    # print('----------------low lidar time stamp----------------')
    # print(low_pc.header.stamp)
    # print(low_pc.header.stamp.nsecs)
    
        

def sub_and_filter():
    rospy.init_node('E_twolidars_pub_node', anonymous=True)
    
    #注册两个雷达的话题
    
    pole_high_sub = message_filters.Subscriber('/e_high/rslidar_points',PointCloud2)
    pole_low_sub = message_filters.Subscriber('/e_low/rslidar_points',PointCloud2)
    
    all_point_filter = message_filters.ApproximateTimeSynchronizer([pole_high_sub,pole_low_sub], queue_size = 10, slop=0.05)
    
    all_point_filter.registerCallback(cat_callback)
    rospy.spin()
    

sub_and_filter()
