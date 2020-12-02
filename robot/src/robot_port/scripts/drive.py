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
	cmd = msg.drivecmd
	move_cmd = Twist()
	if self.is_spin:
	    rospy.logerr("Drive: Spinning now! Please stop spinning first.")
	    return
	if cmd == FRONT:
	    move_cmd.linear.x = max_l
	elif cmd == BACK:
	    move_cmd.linear.x = -max_l
	elif cmd == LEFT:
	    move_cmd.linear.x = max_l
	    move_cmd.angular.z = max_a
	elif cmd == RIGHT:
	    move_cmd.linear.x = max_l
	    move_cmd.angular.z = -max_a
	elif cmd == CLOCKWISE:
	    move_cmd.angular.z = max_a
	elif cmd == ANTICLOCKWISE:
	    move_cmd.angular.z = -max_a
	self.move_pub.publish(move_cmd)

    def spin(self):
	move_cmd = Twist()
	move_cmd.angular.z = max_a
	self.move_pub.publish(move_cmd)

    def start(self):
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
	    if self.is_spin and (self.rb_s.get_status() != rb.run or self.exp_s.get_status() != exp.move):
		self.spin()
	    rate.sleep()
	return


if __name__ == '__main__':
    try:
        d = drive()
        d.start()
    except rospy.ROSInterruptException:
	pass
