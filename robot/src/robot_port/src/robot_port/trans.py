#!/usr/bin/env python


import rospy
import tf
from enum_list import *
from math import cos, sin, pi
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
from robot_port.msg import posi

class trans:

	def __init__(self):
		self.listener = tf.TransformListener()
		rospy.sleep(1)
		global WORLD_FRAME
		WORLD_FRAME = "odom"
		return

	def frame_exists(self, frame_id):
		return self.listener.frameExists(frame_id)

	def rotate(self, x, y, z, rot):
		R = tf.transformations.quaternion_matrix(rot)
		px = R[0][0] * x + R[0][1] * y + R[0][2] * z
		py = R[1][0] * x + R[1][1] * y + R[1][2] * z
		pz = R[2][0] * x + R[2][1] * y + R[2][2] * z
		return [px, py, pz]

	def normalize_angle(self, theta):
		if theta > pi:
			k = int(theta / (2*pi))
			theta -= 2*(k + 1)*pi
		elif theta < -pi:
			k = int((-theta) / (2*pi))
			theta += 2*(k + 1)*pi
		return theta

	def quat_to_angle(self, quat):
		(r, p, y) = tf.transformations.euler_from_quaternion(quat)
		return y

	def get_msg(self):
		try:
			msg = rospy.wait_for_message('/odom', Odometry, timeout = 5)
		except rospy.ROSInterruptException:
			return
		return msg

	def get_transformation(self, frame_a, frame_b, time = None):
		if time is None:
			time = rospy.Time(0)
		return self.listener.lookupTransform("/" + frame_b, "/" + frame_a, time)

	def point_a_to_b(self, x, y, z, *args):
		"Transform the coordinates of a point. You can give two frame_id, and stamp is optional. Also, you can give the transformation directly."
		if len(args) in [2, 3]:
			(trans, rot) = self.get_transformation(*args)
		elif len(args) == 1:
			(trans, rot) = args[0]
		else:
			raise Exception("The number of arguments is wrong!")
		[x, y, z] = self.rotate(x, y, z, rot)
		x += trans[0]
		y += trans[1]
		z += trans[2]
		return [x, y, z]

	def vector_a_to_b(self, x, y, z, *args):
		"Transform the coordinates of a vector. You can give two frame_id, and stamp is optional. Also, you can give the transformation directly."
		if len(args) in [2, 3]:
			(trans, rot) = self.get_transformation(*args)
		elif len(args) == 1:
			(trans, rot) = args[0]
		else:
			raise Exception("The number of arguments is wrong!")
		return self.rotate(x, y, z, rot)

	def angle_a_to_b(self, theta, *args):
		"You can give two frame_id, and stamp is optional. Also, you can give the transformation directly."
		if len(args) in [2, 3]:
			(trans, rot) = self.get_transformation(*args)
		elif len(args) == 1:
			(trans, rot) = args[0]
		else:
			raise Exception("The number of arguments is wrong!")
		theta += self.quat_to_angle(rot)
		return self.normalize_angle(theta)

	def get_posi(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		px = msg.pose.pose.position.x
		py = msg.pose.pose.position.y
		pz = msg.pose.pose.position.z
		WORLD_FRAME = msg.header.frame_id
		[px, py, pz] = self.point_a_to_b(px, py, pz, WORLD_FRAME, VMAP_FRAME)
		return [100 * px, 100 * py]

	def get_velocity(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		vx = msg.twist.twist.linear.x
		vy = msg.twist.twist.linear.y
		vz = msg.twist.twist.linear.z
		WORLD_FRAME = msg.header.frame_id
		[vx, vy, vz] = self.vector_a_to_b(vx, vy, vz, WORLD_FRAME, VMAP_FRAME)
		return [100 * vx, 100 * vy]

	def get_angle(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		theta = self.quat_to_angle([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
								msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		return self.angle_a_to_b(theta, WORLD_FRAME, VMAP_FRAME)

	def get_pose(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		[px, py] = self.get_posi(msg)
		theta = self.get_angle(msg)
		[vx, vy] = self.get_velocity(msg)
		return [px, py, vx, vy, theta]


