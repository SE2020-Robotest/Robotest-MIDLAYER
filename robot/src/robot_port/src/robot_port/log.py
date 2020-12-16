#!/usr/bin/env python


import rospy
import time
from std_msgs.msg import String


class log:
	def __init__(self):
		self.log_pub = rospy.Publisher('log_msg', String, queue_size = 10)
		self.w_log = write_log()

	def loginfo(self, string, *args):
		string = string%args
		self.log_pub.publish("[INFO] " + string)
		self.w_log.log("[INFO] " + string)
		rospy.loginfo(string)

	def logwarn(self, string, *args):
		string = string%args
		self.log_pub.publish("[WARN] " + string)
		self.w_log.log("[WARN] " + string)
		rospy.logwarn(string)

	def logerr(self, string, *args):
		string = string%args
		self.log_pub.publish("[ERROR] " + string)
		self.w_log.log("[ERROR] " + string)
		rospy.logerr(string)

class write_log:
	def __init__(self):
		import os
		self.path = os.path.dirname(os.path.realpath(__file__)) + "/log"
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

