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
from robot_port.msg import response
from robot_port.msg import stop
from robot_port.my_pq import test_my_pq as _test_my_pq
from robot_port.status import test_status as _test_status
from robot_port.grid_graph import test_grid_graph as _test_grid_graph
import robot_port.status as status
from robot_port.trans import trans
from robot_port.log import log
from robot_port.enum_list import *
from Queue import Queue


class test:
	def __init__(self):
		self.res = Queue()
		rospy.init_node('test', anonymous = False)

		self.rb_s = status.rb()
		self.exp_s = status.exp()
		self.tr = trans()
		self.my_log = log()

		self.voice_pub = rospy.Publisher('xfspeech', String, queue_size = 10)
		self.path_pub = rospy.Publisher('path', path, queue_size = 10)
		self.path_ori_pub = rospy.Publisher('path_ori', path_ori, queue_size = 10)
		self.map_pub = rospy.Publisher('virtual_map', vmap, queue_size = 5)
		self.dst_pub = rospy.Publisher('dst', point_2d, queue_size = 5)
		self.mark_pub = rospy.Publisher('mark_vmap', String, queue_size = 5)
		self.stop_pub = rospy.Publisher('stop_exp', stop, queue_size = 2)
		
		rospy.Subscriber('response', response, self.get_response)
		
		
		self.m = vmap()
		self.m.w = 300
		self.m.h = 400
		self.m.obj.append(map_object(0, 13, 114, 30, 110))
		self.m.obj.append(map_object(0, 163, 350, 35, 30))
		self.m.obj.append(map_object(0, 253, 360, 40, 40))
		self.m.obj.append(map_object(0, 273, 190, 15, 120))
		self.m.obj.append(map_object(0, 50, 290, 30, 130))
		self.m.obj.append(map_object(1, 142, 253, 33, 50))
		self.m.obj.append(map_object(1, 130, 113, 50, 50))
		self.m.obj.append(map_object(1, 250, 50, 23, 50))
		
		self.m_cplx = vmap()
		self.m_cplx.w = 300
		self.m_cplx.h = 400
		self.m_cplx.obj.append(map_object(0, 13, 114, 30, 110))
		self.m_cplx.obj.append(map_object(0, 163, 350, 35, 30))
		self.m_cplx.obj.append(map_object(0, 253, 360, 40, 40))
		self.m_cplx.obj.append(map_object(0, 273, 190, 15, 120))
		self.m_cplx.obj.append(map_object(0, 50, 290, 30, 130))
		self.m_cplx.obj.append(map_object(0, 50, 310, 120, 40))
		self.m_cplx.obj.append(map_object(1, 142, 253, 33, 50))
		self.m_cplx.obj.append(map_object(1, 130, 113, 50, 50))
		self.m_cplx.obj.append(map_object(1, 250, 50, 23, 50))
		self.m_cplx.obj.append(map_object(1, 150, 390, 20, 50))
		self.my_log.loginfo("Test: Test Node Initialized!")
		return

	def get_response(self, msg):
		try:
			self.res.put(msg)
		except AttributeError:
			pass

	def wait_for_res(self, timeout = 60.0):
		rate = rospy.Rate(5)
		t = rospy.Time.now().to_sec()
		while True:
			if not self.res.empty():
				res = self.res.get()
				self.res.task_done()
				if res.stamp >= t:
					break
			assert rospy.Time.now().to_sec() - t < timeout, "Timed out while wating for response!"
			rate.sleep()
		return res

	def dist(self, a, b):
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def start_exp(self):
		self.rb_s.Start()

	def stop_exp(self):
		self.exp_s.Wait()
		self.rb_s.Stop()
		self.stop_pub.publish(True)

	def load_map(self, m):
		self.map_pub.publish(m)

	def spin(self):
		self.voice_pub.publish("旋转")

	def stop_spinning(self):
		self.voice_pub.publish("停止旋转。")

	def move_to(self, x, y):
		self.dst_pub.publish(x, y)

	def move_along_path(self, path):
		path_msg = path_ori()
		path_msg.start_time = 0
		path_msg.end_time = 0
		for p in path:
			path_msg.p.append(point_2d(p[0], p[1]))
		self.path_ori_pub.publish(path_msg)

	def face_to_user(self):
		self.voice_pub.publish("看我。")

	def move_to_user(self):
		self.voice_pub.publish("过来。")

	def move_to_ori(self):
		self.voice_pub.publish("回原点。")

	def move_to_the_door(self):
		self.voice_pub.publish("到门边。")

	def mark(self):
		self.mark_pub.publish("mark")

	def test_my_pq(self):
		try:
			_test_my_pq()
			self.my_log.loginfo("Test: Priority Queue test pass!")
		except AssertionError as e:
			self.my_log.logerr("Test: Priority Queue test failed! Details: %s", e)

	def test_status(self):
		try:
			_test_status()
			self.my_log.loginfo("Test: Status test pass!")
		except AssertionError as e:
			self.my_log.logerr("Test: Status test failed! Details: %s", e)

	def test_grid_graph(self):
		try:
			_test_grid_graph()
			self.my_log.loginfo("Test: Grid_graph test pass!")
		except AssertionError as e:
			self.my_log.logerr("Test: Grid_graph test failed! Details: %s", e)

	def test_start_exp(self):
		self.my_log.loginfo("Test: Start the Exp!")
		self.rb_s.Start()
		assert self.rb_s.get_status() == status.rb.init, "%s is wrong!"%status.rbs
		self.my_log.loginfo("Test: Status test pass!")
		self.my_log.loginfo("Test: Start the Exp successfully!")

	def test_stop_exp(self):
		self.my_log.loginfo("Test: Stop the Exp!")
		self.exp_s.Wait()
		self.rb_s.Stop()
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.rbs
		rospy.sleep(2)
		msg = self.tr.get_msg()
		v = self.tr.get_velocity(msg)
		assert abs(msg.twist.twist.angular.z) < 0.1, "Failed to stop moving!"
		assert abs(v[0]) < 0.1 and abs(v[1]) < 0.1, "Failed to stop moving!"
		self.my_log.loginfo("Test: Status test pass!")
		self.my_log.loginfo("Test: Stop the Exp successfully!")

	def test_load_map(self, m):
		self.my_log.loginfo("Test: Load the map!")
		self.load_map(m)
		res = self.wait_for_res(20.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		self.my_log.loginfo("Test: Load the map successfully!")

	def test_spin(self):
		self.my_log.loginfo("Test: Spin!")
		self.spin()
		res = self.wait_for_res(5)
		assert res.node == "Drive", "Wrong response!"
		assert res.response, res.discription
		rospy.sleep(2)
		msg = self.tr.get_msg()
		assert abs(msg.twist.twist.angular.z) > max_a - 0.1, "Failed to spin!"
		self.my_log.loginfo("Test: Spin successfully!")

	def test_stop_spinning(self):
		self.my_log.loginfo("Test: Stop spinning!")
		self.stop_spinning()
		res = self.wait_for_res(5)
		assert res.node == "Drive", "Wrong response!"
		assert res.response, res.discription
		rospy.sleep(2)
		msg = self.tr.get_msg()
		assert abs(msg.twist.twist.angular.z) < max_a / 2, "Failed to stop spinning!"
		self.my_log.loginfo("Test: Stop spinning successfully!")

	def test_move_to(self, x, y, accessible = False):
		self.my_log.loginfo("Test: Move to [%s, %s]!"%(x, y))
		self.move_to(x, y)
		res = self.wait_for_res(60.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		p = self.tr.get_posi()
		if accessible:
			assert (self.dist(p, [x, y]) < accu_eps_d + 0.5), "Failed to arrive the destination!"
			self.my_log.loginfo("Test: Arrive at [%s, %s] successfully!"%(x, y))
		else:
			self.my_log.loginfo("Test: Arrive at [%s, %s] successfully!"%(p[0], p[1]))

	def test_move_along_path(self, path, accessible = False):
		self.my_log.loginfo("Test: Move along path: %s", path)
		self.move_along_path(path)
		res = self.wait_for_res(80.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		p = self.tr.get_posi()
		if accessible:
			assert self.dist(p, path[-1]) < accu_eps_d + 0.5, "Failed to arrive the destination!"
			self.my_log.loginfo("Test: Arrive at %s successfully!", path[-1])
		else:
			self.my_log.loginfo("Test: Arrive at %s successfully!", p)

	def test_face_to_user(self):
		self.my_log.loginfo("Test: Face to the user!")
		self.face_to_user()
		res = self.wait_for_res(10.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		self.my_log.loginfo("Test: Face to the user successfully!")

	def test_move_to_user(self):
		self.my_log.loginfo("Test: Move to the user!")
		self.move_to_user()
		res = self.wait_for_res(60.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		self.my_log.loginfo("Test: Move to the user successfully!")

	def test_move_to_ori(self):
		self.my_log.loginfo("Test: Move to the origin point!")
		self.move_to_ori()
		res = self.wait_for_res(60.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		msg = self.tr.get_msg()
		p = self.tr.get_posi(msg)
		angle = self.tr.get_angle(msg)
		assert (self.dist(p, [0.0, 0.0]) < accu_eps_d + 0.5), "Failed to arrive the origin point!"
		assert abs(angle) < 0.1, "Failed to face to x-axes! angle = %s"%(abs(angle))
		self.my_log.loginfo("Test: Move to the origin point successfully!")

	def test_move_to_the_door(self):
		self.my_log.loginfo("Test: Move to the door!")
		self.move_to_the_door()
		res = self.wait_for_res(60.0)
		assert res.node == "Navi", "Wrong response!"
		assert res.response, res.discription
		assert self.rb_s.get_status() == status.rb.run, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Move to the origin point successfully!")

	def test_mark(self):
		self.my_log.loginfo("Test: Mark the new vmap!")
		self.mark()
		res = self.wait_for_res(5)
		assert res.node == "vmap_broadcaster", "Wrong response!"
		assert res.response, res.discription
		rospy.sleep(2)
		p = self.tr.get_posi()
		assert self.dist(p, [0, 0]) < rough_eps_d / 2, "Failed to mark the vmap!"
		self.my_log.loginfo("Test: Mark the new vmap successfully!")

	def test_tf(self):
		BASE_FRAME = rospy.get_param("base_frame")
		self.my_log.loginfo("Test: TF transform test!")
		assert self.tr.listener.canTransform('/' + VMAP_FRAME, '/' + BASE_FRAME, rospy.Time()), "/vmap frame and /odom cannot transform!"
		self.my_log.loginfo("Test: TF transform between vmap and odom test pass!")
		assert self.tr.listener.canTransform('/' + VMAP_FRAME, '/' + ROBOT_FRAME, rospy.Time()), "/vmap frame and /base_link cannot transform!"
		self.my_log.loginfo("Test: TF transform between vmap and base_link test pass!")
		assert self.tr.listener.canTransform('/' + BASE_FRAME, '/' + ROBOT_FRAME, rospy.Time()), "/odom frame and /base_link cannot transform!"
		self.my_log.loginfo("Test: TF transform between odom and base_link test pass!")

	def test_experiment(self):
		####################################################################
		#   Initial Status Test
		####################################################################
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		
		####################################################################
		#   TF Transform Test
		####################################################################
		self.test_tf()
		
		####################################################################
		#   Start Exp Test
		####################################################################
		self.test_start_exp()
		
		####################################################################
		#   Load Map Test
		####################################################################
		self.test_load_map(self.m)
		
		####################################################################
		#   Spin and Stop Spinning Test
		####################################################################
		self.test_spin()
		self.test_stop_spinning()
		
		####################################################################
		#   Basic Move Test
		####################################################################
		self.test_move_to(299, 0, True)
		
		####################################################################
		#   Mark Vmap Test
		####################################################################
		self.test_mark()
		
		####################################################################
		#   Move To Destination Test
		####################################################################
		self.test_move_to(299, 399, True)
		
		####################################################################
		#   Face To User Test
		####################################################################
		self.test_face_to_user()
		
		####################################################################
		#   Move Along Path Test
		####################################################################
		path = [[299, 399], [250, 350], [200, 250], [150, 150], [100, 100], [50, 50], [0, 0]]
		self.test_move_along_path(path, True)
		
		####################################################################
		#   Move To the Door
		####################################################################
		self.test_move_to_the_door()
		
		####################################################################
		#   Move To the Origin Point Test
		####################################################################
		self.test_move_to_ori()
		
		####################################################################
		#   Move To User Test
		####################################################################
		self.test_move_to(150, 200)
		self.test_move_to_user()
		
		####################################################################
		#   Stop Exp Test
		####################################################################
		self.test_stop_exp()

	def status_transfer_test(self):
		####################################################################
		#   Initial Status Test
		####################################################################
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		
		####################################################################
		#   TF Transform Test
		####################################################################
		self.test_tf()
		
		####################################################################
		#	Before starting Exp:
		#   Test Load map, Spin and Stop Spinning,
		#	Navigation, Face to User, Move to User.
		####################################################################
		self.my_log.loginfo("Test: Load map when sleeping!")
		self.load_map(self.m)
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that loading map when sleeping."
		
		self.my_log.loginfo("Test: Spin when sleeping!")
		self.spin()
		res = self.wait_for_res(5)
		assert res.node == 'Drive', "Wrong response!"
		assert res.response, res.discription
		
		self.my_log.loginfo("Test: Stop spinning when sleeping!")
		self.stop_spinning()
		res = self.wait_for_res(5)
		assert res.node == 'Drive', "Wrong response!"
		assert res.response, res.discription
		
		self.my_log.loginfo("Test: Move to dst when sleeping!")
		self.move_to(150, 200)
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that moving when sleeping."
		
		self.my_log.loginfo("Test: Move along path when sleeping!")
		path = [[299, 399], [250, 350], [200, 250], [150, 150], [100, 100], [50, 50], [0, 0]]
		self.move_along_path(path)
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that moving when sleeping."
		
		self.my_log.loginfo("Test: Face to user when sleeping!")
		self.face_to_user()
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that facing to user when sleeping."
		
		self.my_log.loginfo("Test: Move to user when sleeping!")
		self.move_to_user()
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that moving to user when sleeping."
		
		####################################################################
		#	Initializing Exp:
		#   Test Spin and Stop Spinning, Navigation, 
		#	Face to User, Move to User.
		####################################################################
		self.test_start_exp()
		
		self.my_log.loginfo("Test: Spin when Initializing!")
		self.spin()
		res = self.wait_for_res(5)
		assert res.node == 'Drive', "Wrong response!"
		assert not res.response, "Failed to detected error that spinning when initializing."
		
		self.my_log.loginfo("Test: Stop spinning when Initializing!")
		self.stop_spinning()
		res = self.wait_for_res(5)
		assert res.node == 'Drive', "Wrong response!"
		assert not res.response, "Failed to detected error that spinning when initializing."
		
		self.my_log.loginfo("Test: Move to dst when Initializing!")
		self.move_to(150, 200)
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that moving when initializing."
		
		self.my_log.loginfo("Test: Move along path when Initializing!")
		path = [[299, 399], [250, 350], [200, 250], [150, 150], [100, 100], [50, 50], [0, 0]]
		self.move_along_path(path)
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that moving when initializing."
		
		self.my_log.loginfo("Test: Face to user when Initializing!")
		self.face_to_user()
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that facing to user when initializing."
		
		self.my_log.loginfo("Test: Move to user when Initializing!")
		self.move_to_user()
		res = self.wait_for_res(5)
		assert res.node == 'Navi', "Wrong response!"
		assert not res.response, "Failed to detected error that moving to user when initializing."
		
		####################################################################
		#	Running Exp:
		#   Test Load Map, Spin and Stop Spinning, Navigation
		#	Stop the exp while moving(navigation, face/move to user, spin)
		####################################################################
		self.test_load_map(self.m)
		self.test_spin()
		self.test_stop_spinning()
		self.test_move_to(299, 0, True)
		self.test_mark()
		
		self.my_log.loginfo("Test: Stop the exp while moving!")
		self.move_to(299, 399)
		rospy.sleep(15)
		self.stop_exp()
		res = self.wait_for_res(5)
		assert res.node == "Navi", "Wrong response!"
		assert not res.response, "Failed to stop navigation!"
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		rospy.sleep(2)
		msg = self.tr.get_msg()
		v = self.tr.get_velocity(msg)
		assert abs(v[0]) < 0.1 and abs(v[1]) < 0.1, "Failed to stop moving!"
		assert abs(msg.twist.twist.angular.z) < 0.1, "Failed to stop moving!"
		self.my_log.loginfo("Test: Stop moving successfully")
		
		self.test_start_exp()
		self.test_load_map(self.m)
		self.my_log.loginfo("Test: Stop the exp while facing to user!")
		self.face_to_user()
		rospy.sleep(0.2)
		self.stop_exp()
		res = self.wait_for_res(5)
		assert res.node == "Navi", "Wrong response!"
		assert not res.response, "Failed to stop facing to user!"
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		rospy.sleep(2)
		msg = self.tr.get_msg()
		v = self.tr.get_velocity(msg)
		assert abs(v[0]) < 0.1 and abs(v[1]) < 0.1, "Failed to stop moving!"
		assert abs(msg.twist.twist.angular.z) < 0.1, "Failed to stop moving!"
		self.my_log.loginfo("Test: Stop facing to user successfully")
		
		self.test_start_exp()
		self.test_load_map(self.m)
		self.my_log.loginfo("Test: Stop the exp while moving to user!")
		self.move_to_user()
		rospy.sleep(1)
		self.stop_exp()
		res = self.wait_for_res(5)
		assert res.node == "Navi", "Wrong response!"
		assert not res.response, "Failed to stop moving to user!"
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		rospy.sleep(2)
		msg = self.tr.get_msg()
		v = self.tr.get_velocity(msg)
		assert abs(v[0]) < 0.1 and abs(v[1]) < 0.1, "Failed to stop moving!"
		assert abs(msg.twist.twist.angular.z) < 0.1, "Failed to stop moving!"
		self.my_log.loginfo("Test: Stop moving to user successfully")
		
		self.my_log.loginfo("Test: Stop the exp while spinning!")
		self.spin()
		rospy.sleep(2)
		self.stop_exp()
		assert self.rb_s.get_status() == status.rb.sleep, "%s is wrong!"%status.rbs
		assert self.exp_s.get_status() == status.exp.wait, "%s is wrong!"%status.exps
		self.my_log.loginfo("Test: Status test pass!")
		rospy.sleep(2)
		msg = self.tr.get_msg()
		v = self.tr.get_velocity(msg)
		assert abs(v[0]) < 0.1 and abs(v[1]) < 0.1, "Failed to stop spinning!"
		assert abs(msg.twist.twist.angular.z) < 0.1, "Failed to stop spinning!"
		self.my_log.loginfo("Test: Stop spinning successfully")
		
		####################################################################
		#	Running Exp:
		#   Test Navigation while the destination is not accessible
		####################################################################
		self.test_start_exp()
		self.test_load_map(self.m_cplx)
		self.test_move_to(0, 399)
		self.test_stop_exp()

	def unit_test(self):
		self.my_log.loginfo("Test: Unit test starting!")
		t = rospy.Time.now().to_sec()
		self.test_my_pq()
		self.test_status()
		self.test_grid_graph()
		try:
			self.test_experiment()
			self.my_log.loginfo("Test: Experiment test pass!")
		except AssertionError as e:
			self.my_log.logerr("Test: Experiment test failed! Details: %s", e)
		self.my_log.loginfo("Test: The unit test totally last %s seconds.", rospy.Time.now().to_sec() - t)
		
	def white_test(self):
		self.my_log.loginfo("Test: White test starting!")
		t = rospy.Time.now().to_sec()
		try:
			self.status_transfer_test()
			self.my_log.loginfo("Test: Status transfer test pass!")
		except AssertionError as e:
			self.my_log.logerr("Test: Status transfer test failed! Details: %s", e)
		self.my_log.loginfo("Test: The white test totally last %s seconds.", rospy.Time.now().to_sec() - t)

	def test(self):
		self.unit_test()
		self.white_test()

if __name__ == '__main__':
	try:
		t = test()
		raw_input("Press Enter to Start Test:")
		t.test()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
