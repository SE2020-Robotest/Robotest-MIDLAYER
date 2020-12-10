#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import sqrt
from std_msgs.msg import String
from robot_port.msg import voice_cmd
from robot_port.msg import path_ori
from robot_port.msg import path
from robot_port.msg import point_2d
from robot_port.msg import map_object
from robot_port.msg import vmap
from my_pq import test_my_pq as _test_my_pq
from status import test_status as _test_status
from grid_graph import test_grid_graph as _test_grid_graph
import status
from trans import trans
from enum_list import *


class test:
	def __init__(self):
		self.voice_pub = rospy.Publisher('voice_cmd', voice_cmd, queue_size = 10)
		self.path_pub = rospy.Publisher('path', path, queue_size = 10)
		self.path_ori_pub = rospy.Publisher('path_ori', path_ori, queue_size = 10)
		self.map_pub = rospy.Publisher('virtual_map', vmap, queue_size = 5)
		self.dst_pub = rospy.Publisher('dst', point_2d, queue_size = 5)
		self.mark_pub = rospy.Publisher('mark_vmap', String, queue_size = 5)
		
		rospy.init_node('test', anonymous = False)
		
		self.rb_s = status.rb()
		self.exp_s = status.exp()
		self.tr = trans()
		
		rospy.loginfo("Test: Test Node Initialized!")
		return

	def dist(self, a, b):
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def test_my_pq(self):
		try:
			_test_my_pq()
			rospy.loginfo("Test: Priority Queue test pass!")
		except Exception as e:
			rospy.logerr("Test: Priority Queue test failed! Details: %s", e)

	def test_status(self):
		try:
			_test_status()
			rospy.loginfo("Test: Status test pass!")
		except Exception as e:
			rospy.logerr("Test: Status test failed! Details: %s", e)

	def test_grid_graph(self):
		try:
			_test_grid_graph()
			rospy.loginfo("Test: Grid_graph test pass!")
		except Exception as e:
			rospy.logerr("Test: Grid_graph test failed! Details: %s", e)

	def start_exp(self):
		self.rb_s.Start()

	def stop_exp(self):
		self.exp_s.Wait()
		self.rb_s.Stop()

	def load(self):
		m = vmap()
		m.w = 300
		m.h = 400
		m.obj.append(map_object(0, 13, 114, 30, 110))
		m.obj.append(map_object(0, 163, 350, 35, 30))
		m.obj.append(map_object(0, 253, 360, 40, 40))
		m.obj.append(map_object(0, 273, 190, 15, 120))
		m.obj.append(map_object(0, 50, 290, 30, 130))
		m.obj.append(map_object(1, 142, 253, 33, 50))
		m.obj.append(map_object(1, 130, 113, 50, 50))
		m.obj.append(map_object(1, 250, 50, 23, 50))
		self.map_pub.publish(m)

	def test_spin(self):
		self.voice_pub.publish(rospy.Time.now().secs, "旋转")

	def test_stop_spinning(self):
		self.voice_pub.publish(rospy.Time.now().secs, "停止旋转")

	def test_move_to(self, x, y):
		self.dst_pub.publish(x, y)

	def test_move_along_path(self, path):
		path_msg = path_ori()
		path_msg.start_time = 0
		path_msg.end_time = 0
		for p in path:
			path_msg.p.append(point_2d(p[0], p[1]))
		self.path_ori_pub.publish(path_msg)

	def test_mark(self):
		self.mark_pub.publish("mark")

	def test_experiment(self):
		####################################################################
		#   TF Transform Test
		####################################################################
		rospy.loginfo("Test: TF transform test!")
		assert self.tr.listener.canTransform('/vmap', '/odom', rospy.Time()), "/vmap frame and /odom cannot transform!"
		rospy.loginfo("Test: TF transform between vmap and odom test pass!")
		assert self.tr.listener.canTransform('/vmap', '/base_link', rospy.Time()), "/vmap frame and /base_link cannot transform!"
		rospy.loginfo("Test: TF transform between vmap and base_link test pass!")
		assert self.tr.listener.canTransform('/odom', '/base_link', rospy.Time()), "/odom frame and /base_link cannot transform!"
		rospy.loginfo("Test: TF transform between odom and base_link test pass!")
		
		####################################################################
		#   Start Exp Test
		####################################################################
		rospy.loginfo("Test: Start the Exp!")
		self.start_exp()
		assert self.rb_s.get_status() == status.rb.init, "%s is wrong!"%status.rbs
		rospy.loginfo("Test: Status test pass!")
		rospy.loginfo("Test: Start the Exp successfully!")
		
		####################################################################
		#   Load Map Test
		####################################################################
		rospy.loginfo("Test: Load the map!")
		self.load()
		t = rospy.Time.now().to_sec()
		while self.rb_s.get_status() == status.rb.init:
			assert rospy.Time.now().to_sec() - t < 60.0, "Timed out while loading map!"
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		rospy.loginfo("Test: Status test pass!")
		rospy.loginfo("Test: Load the map successfully!")
		
		####################################################################
		#   Spin and Stop Spinning Test
		####################################################################
		rospy.loginfo("Test: Spin!")
		self.test_spin()
		rospy.sleep(2)
		msg = self.tr.get_msg()
		assert abs(msg.twist.twist.angular.z) > max_a - 0.1, "Failed to spin!"
		rospy.loginfo("Test: Spin successfully!")
		
		rospy.loginfo("Test: Stop spinning!")
		self.test_stop_spinning()
		rospy.sleep(2)
		msg = self.tr.get_msg()
		assert abs(msg.twist.twist.angular.z) < max_a / 2, "Failed to stop spinning!"
		rospy.loginfo("Test: Stop spinning successfully!")
		
		####################################################################
		#   Basic Move Test
		####################################################################
		rospy.loginfo("Test: Move to [299, 0]!")
		self.test_move_to(299, 0)
		rospy.sleep(1)
		t = rospy.Time.now().to_sec()
		while self.exp_s.get_status() == status.exp.move:
			assert rospy.Time.now().to_sec() - t < 40.0, "Timed out while moving!"
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		rospy.loginfo("Test: Status test pass!")
		p = self.tr.get_posi()
		assert self.dist(p, [299, 0]) < eps_d, "Failed to arrive the destination!"
		rospy.loginfo("Test: Arrive at [299, 0] successfully!")
		
		####################################################################
		#   Mark Vmap Test
		####################################################################
		rospy.loginfo("Test: Mark the new vmap!")
		self.test_mark()
		rospy.sleep(1)
		p = self.tr.get_posi()
		assert self.dist(p, [0, 0]) < eps_d / 2, "Failed to mark the vmap!"
		rospy.loginfo("Test: Mark the new vmap successfully!")
		
		####################################################################
		#   Move to Destination Test
		####################################################################
		rospy.loginfo("Test: Move to [299, 399]!")
		self.test_move_to(299, 399)
		rospy.sleep(1)
		t = rospy.Time.now().to_sec()
		while self.exp_s.get_status() == status.exp.move:
			assert rospy.Time.now().to_sec() - t < 70.0, "Timed out while moving!"
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		rospy.loginfo("Test: Status test pass!")
		p = self.tr.get_posi()
		assert (self.dist(p, [299, 399]) < eps_d), "Failed to arrive the destination!"
		rospy.loginfo("Test: Arrive at [299, 399] successfully!")
		
		####################################################################
		#   Move Along Path Test
		####################################################################
		rospy.loginfo("Test: Move along path [299, 399], [250, 350], [200, 250], [150, 150], [100, 100], [50, 50], [0, 0]")
		path = [[299, 399], [250, 350], [200, 250], [150, 150], [100, 100], [50, 50], [0, 0]]
		self.test_move_along_path(path)
		rospy.sleep(1)
		t = rospy.Time.now().to_sec()
		while self.exp_s.get_status() == status.exp.move:
			assert rospy.Time.now().to_sec() - t < 80.0, "Timed out while moving!"
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		rospy.loginfo("Test: Status test pass!")
		p = self.tr.get_posi()
		assert self.dist(p, [0, 0]) < eps_d + 0.5, "Failed to arrive the destination!"
		rospy.loginfo("Test: Arrive at [0, 0] successfully!")
		
		####################################################################
		#   Stop Exp Test
		####################################################################
		rospy.loginfo("Test: Stop the Exp!")
		self.stop_exp()
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.rbs
		rospy.loginfo("Test: Status test pass!")
		rospy.loginfo("Test: Stop the Exp successfully!")

	def start(self):
		rospy.loginfo("Start testing!")
		self.test_my_pq()
		self.test_status()
		self.test_grid_graph()
		try:
			self.test_experiment()
			rospy.loginfo("Test: Experiment test pass!")
		except Exception as e:
			rospy.logerr("Test: Experiment test failed! Details: %s", e)
		rospy.spin()


if __name__ == '__main__':
	try:
		t = test()
		raw_input("Press Enter to Start Test:")
		t.start()
	except rospy.ROSInterruptException:
		pass
