#!/usr/bin/env python
from __future__ import print_function, division
from PIL import Image
import rosbag
import sys

bag_name = '/home/pang/SLAM_THINGS/Datasets/odom_corridor2_2020-01-21-23-27-01.bag'

#topics = ["/odom"]
# bag = rosbag.Bag(bag_name)
# info_name = "odom.txt"

# output_dir = '/home/pang/SLAM_THINGS/Datasets/'
output_dir = bag_name.split((bag_name.split('/')[-1]))[0]
topics = ["/odom"]
bag = rosbag.Bag(bag_name)
info_name =bag_name.split('/')[-1].split('.')[0]
info_name =output_dir + info_name+'.txt'



odom = []
infos = []



for topic, msg, t in bag.read_messages(topics=topics):  # print(topic)# print(msg.header)

    header = msg.header
    header_seq = header.seq
    stamp_sec = header.stamp.secs
    stamp_nsec = header.stamp.nsecs
    frame_id = header.frame_id

    # plc: modify timestamps
    t=str(t)
    time_front = t[:10]
    time_back = t[10:16]
    t = time_front + '.' + time_back
    # t=round(float(t)/1e9,16)
    # t=float(t)/1e9

    # plc: add quaternion after position
    if (topic == str('/odom')):  # inspva info
        print("{} {} {} {} {} {} {} {}".format(t,
                                            msg.pose.pose.position.x,
                                            msg.pose.pose.position.y,
                                            msg.pose.pose.position.z,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w))


        infos.append("{} {} {} {} {} {} {} {}".format(t,
                                                   msg.pose.pose.position.x,
                                                   msg.pose.pose.position.y,
                                                   msg.pose.pose.position.z,
                                                   msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w
                                                   ))

with open(info_name, "w") as fp:
    for it in infos:
        fp.write(it + "\n")
print("Saving path: "+info_name)
bag.close()
