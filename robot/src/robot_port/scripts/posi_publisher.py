#!/usr/bin/env python


import rospy
import tf
from trans import trans
from robot_port.msg import posi

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
	posi_pub = rospy.Publisher('posi_pub', posi, queue_size = 10)
	rospy.init_node('posi_publisher', anonymous = False)

	tr = trans()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		try:
			msg = tr.get_msg()
			p = tr.get_pose(msg)
		except (rospy.exceptions.ROSException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr("Posi_publisher: Cannot get the position!\nDetails: %s", e)
			rospy.loginfo("Posi_publisher: Wait 10 seconds")
			rospy.sleep(10)
			continue
		posi_pub.publish(msg.header.stamp.secs, p[0], p[1], p[2], p[3], p[4])
		rate.sleep()

if __name__ == '__main__':
	try:
		pub()
	except rospy.ROSInterruptException:
		pass
