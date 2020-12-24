#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from robot_port.msg import voice_cmd
from robot_port.msg import path_ori
from robot_port.msg import path as path_now
from robot_port.msg import point_2d
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.enum_list import *
import robot_port.status as status


class fake_cmd:
	def __init__(self):
		self.voice_pub = rospy.Publisher('xfspeech', String, queue_size = 10)
		self.path_pub = rospy.Publisher('path', path_now, queue_size = 10)
		self.path_ori_pub = rospy.Publisher('path_ori', path_ori, queue_size = 10)
		self.map_pub = rospy.Publisher('virtual_map', vmap, queue_size = 5)
		self.dst_pub = rospy.Publisher('dst', point_2d, queue_size = 5)
		self.mark_pub = rospy.Publisher('mark_vmap', String, queue_size = 5)
		rospy.init_node('fake_cmd', anonymous = False)
		rospy.loginfo("Fake_cmd: Fake_cmd Node Initialized!")
		return

	def start(self):
		while not rospy.is_shutdown():
			s = str(raw_input("Input the fake command:"))
			s.strip
			if s == "comm test":
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
				p = path_now([point_2d(0.0, 0.0), point_2d(20.0, 20.0), point_2d(50.0, 50.0)])
				self.path_pub.publish(p)
				self.path_ori_pub.publish(p_o)
			elif s == "load map":
				'''
				map_object:
	    	    	bool type 	# False means CUBE, True means CYLINDER
	    	    	float64 x
	    	   		float64 y
	    	    	float64 w
	    	    	float64 h
				vmap:
	    	    	float32 w
	    	    	float32 h
	    	    	map_object[] obj
				'''
				m = vmap()
				m.w = 260
				m.h = 360
				m.obj.append(map_object(CUBE, 80, 90, 50, 60))
				m.obj.append(map_object(CUBE, 190, 180, 60, 80))
				m.obj.append(map_object(CYLINDER, 150, 260, 40, 50))
				m.obj.append(map_object(CYLINDER, 210, 50, 30, 50))
				self.map_pub.publish(m)
			elif s == "move_dst test":
				self.voice_pub.publish("move to the door")
			elif s == "move_path test":
				path = [[260, 360], [250, 350], [200, 250], [150, 150], [100, 100], [50, 50], [0, 0]]
				path_msg = path_ori()
				path_msg.start_time = rospy.Time.now().secs
				path_msg.end_time = rospy.Time.now().secs
				for p in path:
					path_msg.p.append(point_2d(p[0], p[1]))
				self.path_ori_pub.publish(path_msg)
			elif s == "move to":
				x = float(input("Input the x coordinate:"))
				y = float(input("Input the y coordinate:"))
				self.dst_pub.publish(x, y)
			elif s == "run exp":
				rospy.loginfo("Fake_cmd: Run the Exp! Set the %s to be '%s'.", status.rbs, status.rb.run)
				rospy.set_param(status.rbs, status.rb.run)
			elif s == "start exp":
				rospy.loginfo("Fake_cmd: Initialize the Exp! Set the %s to be '%s'.", status.rbs, status.rb.init)
				rospy.set_param(status.rbs, status.rb.init)
			elif s == "stop exp":
				rospy.loginfo("Fake_cmd: Stop the Exp! Set the %s to be '%s'.", status.rbs, status.rb.sleep)
				rospy.set_param(status.exps, status.exp.wait)
				rospy.set_param(status.rbs, status.rb.sleep)
			elif s == "wait":
				rospy.loginfo("Fake_cmd: Waiting for command! Set the %s to be '%s'.", status.exps, status.exp.wait)
				rospy.set_param(status.exps, status.exp.wait)
			elif s == "move":
				rospy.loginfo("Fake_cmd: Moving! Set the %s to be '%s'.", status.exps, status.exp.move)
				rospy.set_param(status.exps, status.exp.move)
			elif s == "reconize":
				rospy.loginfo("Fake_cmd: Gesture recognition! Set the %s to be '%s'.", status.exps, status.exp.recog)
				rospy.set_param(status.exps, status.exp.recog)
			elif s == "status":
				print rospy.get_param(status.rbs), rospy.get_param(status.exps)
			elif s == "mark":
				self.mark_pub.publish("mark")
			else:
				self.voice_pub.publish(s)
		return


if __name__ == '__main__':
	try:
		f = fake_cmd()
		f.start()
	except rospy.ROSInterruptException:
		pass
