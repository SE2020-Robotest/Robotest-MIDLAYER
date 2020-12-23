#!/usr/bin/env python


import rospy
import tf
from robot_port.log import log
from robot_port.enum_list import *
import geometry_msgs.msg
from std_msgs.msg import String
from robot_port.msg import response

class vmap_coordinate:
	def __init__(self):
		rospy.init_node('vmap_broadcaster', anonymous = False)
		self.response_pub = rospy.Publisher('response', response, queue_size = 10)
		rospy.Subscriber('mark_vmap', String, self.mark_vmap)

		self.t = geometry_msgs.msg.TransformStamped()
		self.marked = False
		self.my_log = log()
		return

	def response(self, discription, response):
		self.response_pub.publish(rospy.Time.now().to_sec(), "vmap_broadcaster", discription, response)

	def mark_vmap(self, msg):
		if msg.data == "mark":
			self.mark()

	def mark(self):
		listener = tf.TransformListener()
		BASE_FRAME = rospy.get_param("base_frame")
		try:
			listener.waitForTransform("/" + ROBOT_FRAME, "/" + BASE_FRAME, rospy.Time(0),rospy.Duration(4.0))
			[trans,rot] = listener.lookupTransform("/" + BASE_FRAME, "/" + ROBOT_FRAME, rospy.Time(0))
		except rospy.ROSInterruptException:
			return False
		except Exception as e:
			self.my_log.logerr("Vmap_broadcaster: Cannot get the Transform!\nDetails: %s", e)
			self.response('Failed to mark the vmap!', False)
			return False
		self.t = geometry_msgs.msg.TransformStamped()
		self.t.header.frame_id = BASE_FRAME
		self.t.header.stamp = rospy.Time.now()
		self.t.child_frame_id = VMAP_FRAME
		self.t.transform.translation.x = trans[0]
		self.t.transform.translation.y = trans[1]
		self.t.transform.translation.z = trans[2]
		self.t.transform.rotation.x = rot[0]
		self.t.transform.rotation.y = rot[1]
		self.t.transform.rotation.z = rot[2]
		self.t.transform.rotation.w = rot[3]
		self.marked = True
		self.response('Mark the vmap successfully!', True)
		return True

	def start(self):
		m = tf.TransformBroadcaster()
		self.mark()
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if self.marked:
				self.t.header.stamp = rospy.Time.now()
				m.sendTransformMessage(self.t)
			rate.sleep()

if __name__ == '__main__':  
	try:
		vmap = vmap_coordinate()
		vmap.start()
	except rospy.ROSInterruptException:
		pass
