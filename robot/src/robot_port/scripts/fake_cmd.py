#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from robot_port.msg import voice_cmd
from robot_port.msg import path_ori
from robot_port.msg import path
from robot_port.msg import point_2d
import status


class fake_cmd:
    def __init__(self):
	self.voice_pub = rospy.Publisher('xfword', voice_cmd, queue_size = 10)
	self.path_pub = rospy.Publisher('send_rb_path', path, queue_size = 10)
	self.path_ori_pub = rospy.Publisher('send_rb_path_ori', path_ori, queue_size = 10)
	rospy.init_node('fake_cmd', anonymous = False)
        rospy.loginfo("Fake_cmd: Fake_cmd Node Initialized!")
	
	return

    def start(self):
	while not rospy.is_shutdown():
	    s = raw_input("Input the fake command:")
	    self.voice_pub.publish(rospy.Time.now().secs, s)
	    if s == "test":
		'''
		path_ori:
	    	    int32 start_time
	    	    int32 end_time
	    	    point_2d[] p
		point_2d:
	    	    float64 x
	    	    float64 y
		'''
        	rospy.loginfo("Fake_cmd: Start the communication test!")
		p_o = path_ori(10, 100, [point_2d(0.0, 0.0), point_2d(10.0, 10.0), point_2d(100.0, 100.0)])
		p = path([point_2d(0.0, 0.0), point_2d(20.0, 20.0), point_2d(50.0, 50.0)])
		self.path_pub.publish(p)
		self.path_ori_pub.publish(p_o)
	    elif s == "spin":
		s = "旋转"
	    elif s == "stop spinning":
		s = "停止旋转"
	    elif s == "run exp":
        	rospy.loginfo("Fake_cmd: Run the Exp! Set the %s to be '%s'.", status.rbs, status.rb.run)
		rospy.set_param(status.rbs, status.rb.run)
	    elif s == "start exp":
        	rospy.loginfo("Fake_cmd: Initialize the Exp! Set the %s to be '%s'.", status.rbs, status.rb.init)
		rospy.set_param(status.rbs, status.rb.init)
	    elif s == "stop exp":
        	rospy.loginfo("Fake_cmd: Stop the Exp! Set the %s to be '%s'.", status.rbs, status.rb.sleep)
		rospy.set_param(status.rbs, status.rb.sleep)
	return


if __name__ == '__main__':
    f = fake_cmd()
    f.start()
