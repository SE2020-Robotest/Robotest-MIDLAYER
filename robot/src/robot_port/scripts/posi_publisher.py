#!/usr/bin/env python


import rospy
import tf
from robot_port.log import log
from robot_port.status import rb
from robot_port.trans import trans
from robot_port.msg import posi
from robot_port.enum_list import *

'''
posi:
	int32 stamp
	float64 x
	float64 y
	float64 vx
	float64 vy
	float64 angle
'''


def pub():
	posi_pub = rospy.Publisher('posi_pub', posi, queue_size = 5)
	rospy.init_node('posi_publisher', anonymous = False)

	tr = trans()
	rb_s = rb()
	my_log = log()
	my_log.loginfo("Posi_publisher: Posi Publisher Initialized!")
	rate = rospy.Rate(pub_rate)
	while not rospy.is_shutdown():
		connected = bool(rospy.get_param("connected"))
		if connected:
			try:
				msg = tr.get_msg()
				p = tr.get_pose(msg)
			except (rospy.exceptions.ROSException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				my_log.logerr("Posi_publisher: Cannot get the position!\nDetails: %s", e)
				my_log.loginfo("Posi_publisher: Wait 10 seconds")
				rospy.sleep(10)
				continue
			posi_pub.publish(msg.header.stamp.to_sec(), p[0], p[1], p[2], p[3], p[4])
		rate.sleep()

if __name__ == '__main__':
	try:
		pub()
	except rospy.ROSInterruptException:
		pass
