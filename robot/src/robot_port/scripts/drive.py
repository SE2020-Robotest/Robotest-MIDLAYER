#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from status import rb, exp
from enum_list import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from robot_port.msg import voice_cmd
from robot_port.msg import enum_type

class drive:
	def __init__(self):
		rospy.init_node('drive', anonymous = False)

		self.rb_s = rb()
		self.exp_s = exp()

		self.move_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
		self.move_cmd = Twist()

		rospy.Subscriber('voice_cmd', voice_cmd, self.recieve_voice)
		rospy.Subscriber('drive_cmd', enum_type, self.recieve_drive_cmd)

		self.is_spin = False
		rospy.loginfo("Drive: Drive Node Initialized!")
		return

	def recieve_voice(self, msg):
		'''
		voice_cmd:
	    	int32 stamp
	    	string cmd
		'''
		cmd = msg.cmd
		if cmd == "旋转":
			rospy.loginfo("Drive: Start Spinning!")
			self.is_spin = True
			self.move_cmd = Twist()
		elif cmd == "停止旋转":
			rospy.loginfo("Drive: Stop Spinning!")
			self.is_spin = False

	def recieve_drive_cmd(self, msg):
		'''
		FRONT = 0
		BACK = 1
		LEFT = 2
		RIGHT = 3
		CLOCKWISE = 4
		ANTICLOCKWISE = 5
		'''
		if self.rb_s.get_status() == rb.run and self.exp_s.get_status() == exp.move:
			return
		cmd = msg.type
		move_cmd = Twist()
		if self.is_spin:
			rospy.logerr("Drive: Spinning now! Please stop spinning first.")
			return
		linear = 0
		angular = 0
		if cmd == FRONT:
			linear = max_l
		elif cmd == BACK:
			linear = -max_l
		elif cmd == LEFT:
			linear = max_l
			angular = max_a
		elif cmd == RIGHT:
			linear = max_l
			angular = -max_a
		elif cmd == CLOCKWISE:
			angular = max_a
		elif cmd == ANTICLOCKWISE:
			angular = -max_a
		self.move(linear, angular)

	def spin(self):
		self.move(0, max_a)

	def move(self, linear, angular):
		'''
		Normalize the velocities and publish them.
		The linear velocity and its difference is bounded by max_l and max_delta_l.
		The angular velocity and its difference is bounded by max_a and max_delta_a.
		'''
		if self.rb_s.get_status() == rb.run and self.exp_s.get_status() == exp.move:
			return
		linear = max(min(linear, max_l), -max_l)
		angular = max(min(angular, max_a), -max_a)
		delta_l = linear - self.move_cmd.linear.x
		delta_l = max(min(delta_l, max_delta_l), -max_delta_l)
		delta_a = angular - self.move_cmd.angular.z
		delta_a = max(min(delta_a, max_delta_a), -max_delta_a)
		self.move_cmd.linear.x += delta_l
		self.move_cmd.angular.z += delta_a
		self.move_pub.publish(self.move_cmd)

	def start(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.is_spin:
				self.spin()
			rate.sleep()
		return

if __name__ == '__main__':
	try:
		d = drive()
		d.start()
	except rospy.ROSInterruptException:
		pass
