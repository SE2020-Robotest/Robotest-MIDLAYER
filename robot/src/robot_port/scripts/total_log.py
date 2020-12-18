#!/usr/bin/env python

import rospy
from robot_port.log import write_log
from std_msgs.msg import String


class total_log:
	def __init__(self):
		rospy.init_node('total', anonymous = False)

		self.log = write_log()

		rospy.Subscriber('log_msg', String, self.log_msg)
		rospy.on_shutdown(self.shutdown)
		rospy.loginfo("Log: Log Node Initialized!")
		return

	def log_msg(self, msg):
		self.log.log(msg.data)

	def shutdown(self):
		if rospy.has_param("start_time"):
			rospy.delete_param("start_time")


if __name__ == '__main__':
	try:
		l = total_log()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
