#!/usr/bin/env python


import rospy
import time
from std_msgs.msg import String


class log:
	def __init__(self, if_pub = True):
		self.if_pub = if_pub
		if self.if_pub:
			self.log_pub = rospy.Publisher('log_msg', String, queue_size = 10)
		self.w_log = write_log()

	def loginfo(self, string, *args):
		if isinstance(string, list):
			for s in string:
				self.loginfo(s)
			return
		assert isinstance(string, str), "Argument string is not the type of str!"
		string = string%args
		if self.if_pub:
			self.log_pub.publish("[INFO] " + string)
		self.w_log.log("[INFO] " + string)
		rospy.loginfo(string)

	def logwarn(self, string, *args):
		assert isinstance(string, str), "Argument string is not the type of str!"
		string = string%args
		if self.if_pub:
			self.log_pub.publish("[WARN] " + string)
		self.w_log.log("[WARN] " + string)
		rospy.logwarn(string)

	def logerr(self, string, *args):
		assert isinstance(string, str), "Argument string is not the type of str!"
		string = string%args
		if self.if_pub:
			self.log_pub.publish("[ERROR] " + string)
		self.w_log.log("[ERROR] " + string)
		rospy.logerr(string)

class write_log:
	def __init__(self):
		import os
		if rospy.has_param("pkg_path"):
			self.path = rospy.get_param("pkg_path")
		else:
			self.path = os.path.dirname(os.path.realpath(__file__))
		self.path += "/log"
		if not os.path.exists(self.path):
			os.makedirs(self.path)
		start_time = time.strftime("%Y-%m-%d %H:%M:%S ", time.localtime())
		if not rospy.has_param("start_time"):
			rospy.set_param("start_time", start_time)
		self.path += "/" + rospy.get_param("start_time")
		if not os.path.exists(self.path):
			os.makedirs(self.path)
		self.path += rospy.get_name() + "_log.log"
		self.ok = True
		try:
			with open(self.path, 'w') as f:
				pass
		except Exception as e:
			self.ok = False
			print e

	def log(self, string):
		if not self.ok:
			return
		with open(self.path, 'a') as f:
			f.write(time.strftime("%Y-%m-%d %H:%M:%S ", time.localtime()) + string + "\n")

