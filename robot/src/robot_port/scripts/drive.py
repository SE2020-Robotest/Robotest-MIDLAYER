#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from robot_port.msg import voice_cmd

class drive:
    def __init__(self):
	self.move_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
	self.is_spin = False
	rospy.init_node('drive', anonymous = False)
        rospy.loginfo("Drive: Drive Node Initialized!")
	
	rospy.Subscriber('xfword', voice_cmd, self.recieve_voice)
	return

    def recieve_voice(self, msg):
	'''
	voice_cmd:
	    int32 stamp
	    string cmd
	'''
	cmd = msg.cmd
	if cmd == "spin":
	    rospy.loginfo("Drive: Start Spinning!")
	    self.is_spin = True
	elif cmd == "stop spin":
	    rospy.loginfo("Drive: Stop Spinning!")
	    self.is_spin = False

    def spin(self):
	move_cmd = Twist()
	move_cmd.angular.z = 5
	self.move_pub.publish(move_cmd)

    def start(self):
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
	    if self.is_spin:
		self.spin()
	    rate.sleep()
	return


if __name__ == '__main__':
    d = drive()
    d.start()
