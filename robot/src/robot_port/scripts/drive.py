#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from robot_port.status import rb, exp
from robot_port.log import log
from robot_port.enum_list import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from robot_port.msg import voice_cmd
from robot_port.msg import enum_type
from robot_port.msg import response
from robot_port.msg import stop

class drive:
	def __init__(self):
		rospy.init_node('drive', anonymous = False)

		self.rb_s = rb()
		self.exp_s = exp()
		self.my_log = log()

		self.move_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
		self.response_pub = rospy.Publisher('response', response, queue_size = 10)
		self.move_cmd = Twist()

		rospy.Subscriber('voice_cmd', voice_cmd, self.receive_voice)
		rospy.Subscriber('drive_cmd', enum_type, self.receive_drive_cmd)
		rospy.Subscriber('stop_exp', stop, self.stop_exp)

		self.is_spin = False
		self.my_log.loginfo("Drive: Drive Node Initialized!")
		return

	def stop_exp(self, msg):
		if msg.stop:
			self.is_spin = False

	def response(self, discription, response):
		self.response_pub.publish(rospy.Time.now().to_sec(), "Drive", discription, response)

	def canDrive(self):
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		return rb_status == rb.sleep or (rb_status == rb.run and exp_status == exp.wait)

	def receive_voice(self, msg):
		'''
		voice_cmd:
	    	int32 stamp
	    	string cmd
		'''
		cmd = msg.cmd
		if cmd in v_cmd["spin"]:
			if not self.canDrive():
				self.my_log.logerr("Drive: Cannot start spinning now!")
				self.response("Cannot start spinning now!", False)
				self.is_spin = False
			else:
				self.my_log.loginfo("Drive: Start spinning!")
				self.response("Start Spinning!", True)
				self.is_spin = True
		elif cmd in v_cmd["stop spinning"]:
			if not self.canDrive():
				self.my_log.logwarn("Drive: Cannot stop spinning now!")
				self.response("It's moving now!", False)
			else:
				self.my_log.loginfo("Drive: Stop spinning!")
				self.response("Stop Spinning!", True)
			self.is_spin = False

	def receive_drive_cmd(self, msg):
		'''
		FRONT = 0
		BACK = 1
		LEFT = 2
		RIGHT = 3
		CLOCKWISE = 4
		ANTICLOCKWISE = 5
		'''
		if not self.canDrive():
			return
		cmd = msg.type
		move_cmd = Twist()
		if self.is_spin:
			self.my_log.logerr("Drive: Spinning now! Please stop spinning first.")
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
		if not self.canDrive():
			self.move_cmd = Twist()
			self.spin = False
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
