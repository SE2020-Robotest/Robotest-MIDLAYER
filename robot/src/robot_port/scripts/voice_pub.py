#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import re
from robot_port.log import log
from robot_port.enum_list import *
from std_msgs.msg import String
from robot_port.msg import voice_cmd

voice_pub = None

def receive_voice(msg):
	cmd = msg.data
	cmd.rstrip()
	if cmd[-3:] in illegal_suffix:
		cmd = cmd[:-3]
	if not cmd in cmd_list:
		return
	my_log.loginfo(cmd)
	voice_pub.publish(rospy.Time.now().to_sec(), cmd)

def init():
	rospy.init_node('voice_pub', anonymous = False)
	global voice_pub
	voice_pub = rospy.Publisher('voice_cmd', voice_cmd, queue_size = 10)
	global my_log
	my_log = log(if_pub = False)
	rospy.Subscriber('xfspeech', String, receive_voice)


if __name__ == '__main__':
	try:
		init()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

