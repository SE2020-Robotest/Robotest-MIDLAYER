#!/usr/bin/env python


import rospy
import tf
from math import cos, sin, pi
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
from robot_port.msg import posi

class trans:

	def __init__(self):
		self.listener = tf.TransformListener()
		rospy.sleep(1)
		return

	def rotate(self, x, y, theta):
		px = x*cos(theta) - y*sin(theta)
		py = x*sin(theta) + y*cos(theta)
		return [px, py]

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

	def _point_a_to_b(self, frame_a, frame_b, x, y):
		"Transform the coordinates of a point in frame_a into frame_b."
		(trans,rot) = self.listener.lookupTransform("/" + frame_b, "/" + frame_a, rospy.Time(0))
		theta = self.quat_to_angle(rot)
		[x, y] = self.rotate(x, y, theta)
		x += trans[0]
		y += trans[1]
		return [x, y]

	def _vector_a_to_b(self, frame_a, frame_b, x, y):
		"Transform the coordinates of a vector in frame_a into frame_b. Vector just need to be rotated."
		(trans,rot) = self.listener.lookupTransform("/" + frame_b, "/" + frame_a, rospy.Time(0))
		theta = self.quat_to_angle(rot)
		return self.rotate(x, y, theta)

	def _angle_a_to_b(self, frame_a, frame_b, theta):
		"Transform the coordinates of a vector in frame_a into frame_b. Vector just need to be rotated."
		(trans,rot) = self.listener.lookupTransform("/" + frame_b, "/" + frame_a, rospy.Time(0))
		theta += self.quat_to_angle(rot)
		return self.normalize_angle(theta)

	def get_posi(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		px = msg.pose.pose.position.x
		py = msg.pose.pose.position.y
		[px, py] = self.point_world_to_vmap(px, py)
		return [100 * px, 100 * py]

	def get_angle(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		theta = self.quat_to_angle([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
								msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		return self.angle_world_to_vmap(theta)

	def get_velocity(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		vx = msg.twist.twist.linear.x
		vy = msg.twist.twist.linear.y
		return self.vector_robot_to_vmap(100 * vx, 100 * vy)

	def get_pose(self, msg = None):
		if msg is None:
			msg = self.get_msg()
		[px, py] = self.get_posi(msg)
		theta = self.get_angle(msg)
		[vx, vy] = self.get_velocity(msg)
		return [px, py, vx, vy, theta]

	def point_world_to_robot(self, x, y):
		return self._point_a_to_b("odom", "base_link", x, y)

	def vector_world_to_robot(self, x, y):
		return self._vector_a_to_b("odom", "base_link", x, y)

	def point_robot_to_world(self, x, y):
		return self._point_a_to_b("base_link", "odom", x, y)

	def vector_robot_to_world(self, x, y):
		return self._vector_a_to_b("base_link", "odom", x, y)

	def point_vmap_to_robot(self, x, y):
		return self._point_a_to_b("vmap", "base_link", x, y)

	def vector_vmap_to_robot(self, x, y):
		return self._vector_a_to_b("vmap", "base_link", x, y)

	def point_robot_to_vmap(self, x, y):
		return self._point_a_to_b("base_link", "vmap", x, y)

	def vector_robot_to_vmap(self, x, y):
		return self._vector_a_to_b("base_link", "vmap", x, y)

	def point_vmap_to_world(self, x, y):
		return self._point_a_to_b("vmap", "odom", x, y)

	def vector_vmap_to_world(self, x, y):
		return self._vector_a_to_b("vmap", "odom", x, y)

	def point_world_to_vmap(self, x, y):
		return self._point_a_to_b("odom", "vmap", x, y)

	def vector_world_to_vmap(self, x, y):
		return self._vector_a_to_b("odom", "vmap", x, y)

	def angle_world_to_vmap(self, theta):
		return self._angle_a_to_b("odom", "vmap", theta)

