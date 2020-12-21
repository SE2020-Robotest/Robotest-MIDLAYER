#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import re
from robot_port.log import log
from robot_port.enum_list import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from robot_port.msg import voice_cmd
from roslib import message

real_sense_pub = None

def receive_voice(msg):
	cmd = msg.cmd
	if cmd in v_cmd["move to there"]:
		real_sense_pub.publish()

def read_depth(width, height, data) :
	# read function
	if (height >= data.height) or (width >= data.width) :
		return -1
	data_out = point_cloud2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height]])
	int_data = next(data_out)
	rospy.loginfo("int_data " + str(int_data))
	return int_data

def print_depth(data):
	print data.height, data.width, data.header.frame_id
	read_depth(400, 300, data)

def init():
	rospy.init_node('real_sense_pub', anonymous = False)
	global real_sense_pub
	real_sense_pub = rospy.Publisher('voice_cmd', voice_cmd, queue_size = 1)
	rospy.Subscriber('voice_cmd', voice_cmd, receive_voice)
	rospy.Subscriber('/camera/depth/points', PointCloud2, print_depth)


if __name__ == '__main__':
	try:
		init()
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass

