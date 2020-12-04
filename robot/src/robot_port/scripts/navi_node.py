#!/usr/bin/env python


import rospy
from status import rb, exp
from enum_list import *
import grid_graph as graph
from trans import trans
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import pi, atan2, sqrt, cos
from enum import Enum
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import path
from robot_port.msg import point_2d
from robot_port.msg import enum_type

'''
The origin point is on the bottom-left, and unit is cm

i           y
|           |
|        &  |
|_____ j    |_____ x

This is the corresbonding relation between grid_graph and virtual_map
'''


class navi_method(Enum):
	cmd_vel = 0
	cmd_vel_smooth = 1
	move_base_grid_graph = 2
	move_base_map_server = 3

cur_navi_method = navi_method.cmd_vel

class navi_nodes:
	def __init__(self):
		rospy.init_node('navi_node', anonymous = False)

		self.map_loaded = False
		self.g = None
		self.rb_s = rb()
		self.exp_s = exp()
		self.trans = trans()
		rospy.loginfo("Navi: Navigation Node Initialized!")

		self.move_cmd_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.path_pub = rospy.Publisher('path', path, queue_size = 10)
		self.res_to_ctrl_pub = rospy.Publisher('response_to_ctrl', enum_type, queue_size = 10)

		rospy.Subscriber('virtual_map', vmap, self.load_map)
		rospy.Subscriber('path', path, self.recieve_path)
		rospy.Subscriber('dst', point_2d, self.recieve_dst)

		r = rospy.get_param("robot_radius")
		d = rospy.get_param("grid_size")
		graph.r_robot = float(r)
		graph.delta = float(d)

		return

	def start(self):
		"Start running"
		rospy.spin()
		return

	def load_map(self, msg):
		"Load the map message from Control port"
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
		rb_status = self.rb_s.get_status()
		rospy.loginfo("Navi: Loading the map")
		if rb_status != rb.init:
			rospy.logerr("Navi: Cannot init the map! The Exp hasn't been initialized.")
			return
		self.g = graph.grid_graph(msg.w, msg.h)
		for obj in msg.obj:
			if obj.type == CYLINDER:
				self.g.add_circle(obj.x, obj.y, obj.w)
			elif obj.type == CUBE:
				self.g.add_rect(obj.x, obj.y, obj.w, obj.h)
		self.g.finish()
		cur_p = self.trans.get_posi()
		if not self.g.is_empt_coordinate(cur_p[0], cur_p[1]):
			rospy.logwarn("Navi: Current position in the virtual map is illegal!")
			p = self.g.correct_point_coordinate(cur_p[0], cur_p[1])
			rospy.loginfo("Navi: Automatically move to %s, %s", p[0], p[1])
			if not self.move_to_by_cmd_vel(p):
				rospy.logerr("Navi: Cannot correct the position automatically!")
				rospy.logerr("Navi: The Exp failed to initialize!")
				self.rb_s.Stop()
				self.res_to_ctrl_pub.publish(ERROR)
				return
		rospy.loginfo("Navi: Load the map successfully!")
		self.res_to_ctrl_pub.publish(FINISHED)
		self.rb_s.Run()								# (Important!) Change the status
		return

	def recieve_path(self, msg):
		"Recieve the path, correct them and move along this path. Finally, send the corrected path to Control port"
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status != rb.run or exp_status != exp.wait:
			return
		self.exp_s.Move() 							# (Important!) Change the status

	
		return

	def recieve_dst(self, msg):
		"Recieve the destination, correct it and find a path to move along. Finally, send the path to Control port"
		'''
		path:
			point_2d[] p
		point_2d:
			float64 x
			float64 y
		'''
	
		# Check the status
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status != rb.run or exp_status != exp.wait:
			return
		self.exp_s.Move() 							# (Important!) Change the status

		qx = msg.x # destination
		qy = msg.y # destination

		# Get the Robot's position
		try:
			[px, py] = self.trans.get_posi() # origin position
		except rospy.ROSInterruptException:
			return
		except:
			rospy.logerr("Navi: Cannot get the Robot's position!")
			self.exp_s.Wait() 						# (Important!) Change the status
			return

		# Find path
		path = self.g.find_path(px, py, qx, qy)
		if len(path) == 0:
			rospy.logerr("Navi: The destination is not accessable!")
			self.exp_s.Wait() 						# (Important!) Change the status
			return
		rospy.loginfo("Navi: Found the path!")

		# Send path message
		path_msg = []
		path_msg.append(point_2d(px, py)) # origin
		for p in path:
			path_msg.append(point_2d(p[0], p[1]))
		self.path_pub.publish(path_msg)

		# Move to the destination
		rospy.loginfo("Navi: Start moving!")
		if self.move(path):
			rospy.loginfo("Navi: Arrive at destination!")
		else:
			rospy.logerr("Navi: Failed to achieve destination!")
		self.exp_s.Wait() 							# (Important!) Change the status
		return

	def move(self, path):
		if cur_navi_method == navi_method.cmd_vel:
			return self.move_by_cmd_vel(path)
		elif cur_navi_method == navi_method.cmd_vel_smooth:
			return self.move_by_cmd_vel_smooth(path)
		elif cur_navi_method == navi_method.move_base_grid_graph:
			return self.move_by_move_base_grid_graph(path)
		elif cur_navi_method == navi_method.move_base_map_server:
			return self.move_by_move_base_map_server(path)

	def move_by_cmd_vel(self, path):
		for p in path:
			if self.exp_s.get_status() != exp.move:
				return False
			if not self.move_to_by_cmd_vel(p):
				return False
		return True

	def move_by_cmd_vel_smooth(self, path):
		l = len(path)
		for i in range(l):
			if self.exp_s.get_status() != exp.move:
				return False
			if not self.move_to_by_cmd_vel_smooth(path[i - 1] if i > 0 else None, path[i], i < l - 1):
				return False
		return True

	def move_to_by_cmd_vel_smooth(self, last_p, p, has_next = False):
		rate = rospy.Rate(10)
		'''
		print "Navi: Move to %s, %s"%(p[0], p[1])
		
		while True:
			if self.exp_s.get_status() != exp.move and self.rb_s.get_status() != rb.init:
				return False
			msg = self.trans.get_msg()
			cur_p = self.trans.get_posi(msg)
			cur_angle = self.trans.get_angle(msg)
			angle = self.trans.normalize_angle(self.get_orientation(cur_p, p) - cur_angle)
			if abs(angle) < eps_a:
				break
			if has_next and :
				break
			if abs(angle) > 1.0:
				linear = 0
			else:
				linear = max_l / 2
			angular = 4 * angle
			angular = min(max_a, max(-max_a, angular))
			move_cmd = Twist()
			move_cmd.linear.x = linear
			move_cmd.angular.z = angular
			self.move_cmd_pub.publish(move_cmd)
			rate.sleep()
		
		while True:
			if self.exp_s.get_status() != exp.move and self.rb_s.get_status() != rb.init:
				return False
			msg = self.trans.get_msg()
			cur_p = self.trans.get_posi(msg)
			cur_angle = self.trans.get_angle(msg)
			d = self.dist(p, cur_p)
			angle = self.trans.normalize_angle(self.get_orientation(cur_p, p) - cur_angle)
			if has_next:
				if d < d_turn:
					break
			else:
				if d < eps_d:
					break
			if d*abs(angle) > eps_d:
				angular = angle
			else:
				angular = 0
			angular = min(max_a, max(-max_a, angular))
			
			if abs(angle) > 1.0:
				linear = 0
			elif abs(angle) > 0.5:
				if has_next:
					linear = max_l / 2
				else:
					linear = d / 200
			else:
				if has_next:
					if d < d_slow:
						linear = max_l / 2
					else:
						linear = max_l
				else:
					linear = d / 100
			linear = min(max_l, linear)
			move_cmd = Twist()
			move_cmd.linear.x = linear
			move_cmd.angular.z = angular
			self.move_cmd_pub.publish(move_cmd)
			rate.sleep()

		print "Navi: Arrive at %s, %s"%(cur_p[0], cur_p[1])
		'''
		return True

	def move_to_by_cmd_vel(self, p):
		rate = rospy.Rate(10)
		msg = self.trans.get_msg()
		cur_p = self.trans.get_posi(msg)

		print "Navi: Move to %s, %s"%(p[0], p[1])
		print "Navi: Rotate!"

		while True:
			if self.exp_s.get_status() != exp.move and self.rb_s.get_status() != rb.init:
				return False
			msg = self.trans.get_msg()
			cur_angle = self.trans.get_angle(msg)
			angle = self.trans.normalize_angle(self.get_orientation(cur_p, p) - cur_angle)
			if abs(angle) < eps_a:
				self.move_cmd_pub.publish(Twist())
				break
			linear = 0
			angular = 4 * angle
			angular = min(max_a, max(-max_a, angular))
			move_cmd = Twist()
			move_cmd.linear.x = linear
			move_cmd.angular.z = angular
			self.move_cmd_pub.publish(move_cmd)
			rate.sleep()

		print "Navi: Move!"

		while True:
			if self.exp_s.get_status() != exp.move and self.rb_s.get_status() != rb.init:
				return False
			msg = self.trans.get_msg()
			cur_p = self.trans.get_posi(msg)
			cur_angle = self.trans.get_angle(msg)
			d = self.dist(p, cur_p)
			angle = self.trans.normalize_angle(self.get_orientation(cur_p, p) - cur_angle)
			if d < eps_d:
				break
			if d*abs(angle) > eps_d:
				angular = 2 * angle
			else:
				angular = 0
			angular = min(max_a, max(-max_a, angular))
			if abs(angle) > 1.0:
				linear = 0
			elif abs(angle) > 0.5:
				linear = d / 200
			else:
				linear = d / 100
			linear = min(max_l, linear)
			move_cmd = Twist()
			move_cmd.linear.x = linear
			move_cmd.angular.z = angular
			self.move_cmd_pub.publish(move_cmd)
			rate.sleep()

		print "Navi: Arrive at %s, %s"%(cur_p[0], cur_p[1])

		return True

	def move_by_move_base_grid_graph(self, path):
		pose = self.path_to_pose(path)
		for p in pose:
			if self.exp_s.get_status() != exp.move:
				return
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'vmap'
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose = p
			self.move_base.send_goal(goal)
			try:
				finished_within_time = self.move_base.wait_for_result(rospy.Duration(30))
			except rospy.ROSInterruptException:
				return False
			except rospy.exceptions.ROSException as e:
				rospy.logerr("Posi_publisher: Cannot achieve the goal!\nDetails: %s", e)
				return False
			if finished_within_time:
				rospy.logerr("Posi_publisher: Timed out achieving the goal!")
				return False
			else:
				if self.move_base.get_state() != GoalStatus.SUCCEEDED:
					rospy.logerr("Posi_publisher: Cannot achieve the goal!")
					return False
		rospy.loginfo("Arrive at %s, %s", p.position.x, p.position.y)
		return True

	def move_by_move_base_map_server(self, path):
		return

	def get_orientation(self, p1, p2):
		return atan2(p2[1] - p1[1], p2[0] - p1[0])

	def dist(self, a, b):
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def path_to_pose(self, path):
		dst_pose = []
		last_p = None
		for p in path:
			if last_p is not None:
				dst_pose[-1].orientation = quaternion_from_euler(0, 0, self.get_orientation(last_p, p), axes = 'sxyz')
			dst_pose.append(Pose(Point(p[0], p[1], 0), Quaternion()))
			last_p = p
		dst_pose[-1].orientation = dst_pose[-2].orientation
		return dst_pose


if __name__ == '__main__':
	try:
		navi = navi_nodes()
		navi.start()
	except rospy.ROSInterruptException:
		pass
