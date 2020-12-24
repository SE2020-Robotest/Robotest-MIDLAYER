#!/usr/bin/env python


import rospy
from robot_port.status import rb, exp
from robot_port.enum_list import *
import robot_port.grid_graph as graph
from robot_port.trans import trans
from robot_port.log import log
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import pi, atan2, sqrt, cos
from enum import Enum
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import path as path_now
from robot_port.msg import path_ori
from robot_port.msg import point_2d
from robot_port.msg import voice_cmd
from robot_port.msg import enum_type
from robot_port.msg import response

'''
The origin point is on the bottom-left, and unit is cm

i           y
|           |
|        &  |
|_____ j    |_____ x

This is the corresbonding relation between grid_graph and virtual_map
'''


class navi_nodes:
	def __init__(self):
		rospy.init_node('navi_node', anonymous = False)

		self.map_loaded = False
		self.g = None
		self.rb_s = rb()
		self.exp_s = exp()
		self.trans = trans()
		self.my_log = log()
		self.p_user = self.get_user_posi()

		self.move_cmd_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
		self.move_cmd = Twist()
		self.path_pub = rospy.Publisher('path', path_now, queue_size = 10)
		self.res_to_ctrl_pub = rospy.Publisher('response_to_ctrl', enum_type, queue_size = 10)
		self.response_pub = rospy.Publisher('response', response, queue_size = 10)

		rospy.Subscriber('virtual_map', vmap, self.load_map)
		rospy.Subscriber('path_ori', path_ori, self.receieve_path)
		rospy.Subscriber('dst', point_2d, self.receieve_dst)
		rospy.Subscriber('voice_cmd', voice_cmd, self.receive_voice)
		
		self.my_log.loginfo("Navi: Navigation Node Initialized!")
		return

	def start(self):
		"Start running"
		rospy.spin()
		return

	def response(self, discription, response):
		self.response_pub.publish(rospy.Time.now().to_sec(), "Navi", discription, response)

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
		self.my_log.loginfo("Navi: Loading the map")
		if rb_status != rb.init:
			self.my_log.logerr("Navi: Cannot init the map! The Exp hasn't been initialized.")
			self.response("Cannot init the map! The Exp hasn't been initialized.", False)
			return
		self.g = graph.grid_graph(msg.w, msg.h, self.my_log)
		for obj in msg.obj:
			if obj.type == CYLINDER:
				self.g.add_circle(obj.x, obj.y, obj.w)
			elif obj.type == CUBE:
				self.g.add_rect(obj.x, obj.y, obj.w, obj.h)
		self.g.finish()
		try:
			cur_p = self.trans.get_posi()
		except rospy.exceptions.ROSException as e:
			self.my_log.logerr("Navi: Cannot get the position!\nDetails: %s", e)
			self.rb_s.Stop()
			self.response("Cannot get the position!", False)
			return
		if not self.g.is_empt_coordinate(cur_p[0], cur_p[1]):
			self.my_log.logwarn("Navi: Current position in the virtual map is illegal!")
			p = self.g.correct_point_coordinate(cur_p[0], cur_p[1])
			self.my_log.loginfo("Navi: Automatically move to %s, %s", p[0], p[1])
			if not self.move_to(p):
				self.my_log.logerr("Navi: Cannot correct the position automatically!")
				self.my_log.logerr("Navi: The Exp failed to initialize!")
				self.rb_s.Stop()
				self.res_to_ctrl_pub.publish(ERROR)
				self.response("Cannot correct the position automatically!", False)
				return
		self.my_log.loginfo("Navi: Load the map successfully!")
		self.res_to_ctrl_pub.publish(FINISHED)
		self.rb_s.Run()								# (Important!) Change the status
		self.response("Load the map successfully!", True)
		return

	def receieve_dst(self, msg):
		"Receive the destination"
		try:
			is_finished = self.navi_move_to([msg.x, msg.y])
			if is_finished:
				self.my_log.loginfo("Navi: Move to the destination successfully!")
				self.response("Move along the path successfully!", True)
			else:
				self.my_log.logerr("Navi: Failed to move to the destination!")
				self.response("Failed to move to the destination!", False)
		except rospy.ROSInterruptException:
			return

	def receieve_path(self, msg):
		"receive the origin path"
		ori_path = []
		for p in msg.p:
			ori_path.append([p.x, p.y])
		try:
			is_finished = self.navi_move_along(ori_path)
			if is_finished:
				self.my_log.loginfo("Navi: Move along the path successfully!")
				self.response("Move along the path successfully!", True)
			else:
				self.my_log.logerr("Navi: Failed to move along the path!")
				self.response("Failed to move along the path!", False)
		except rospy.ROSInterruptException:
			return

	def receive_voice(self, msg):
		'''
		voice_cmd:
	    	float64 stamp
	    	string cmd
		'''
		try:
			self.navi_cmd(msg.cmd)
		except rospy.ROSInterruptException:
			return

	def navi_move_to(self, q, eps_d = -1):
		"Navi to the destination: correct it and find a path to move along. Finally, send the path to Control port"
		'''
		parameters:
			q: the destination.
			eps_d: the tollerance of the destination. Default is -1. If eps_d <= 0, then it will be set as accu_eps_d.
		msgs:
			path:
				point_2d[] p
			point_2d:
				float64 x
				float64 y
		'''
		if eps_d <= 0:
			eps_d = accu_eps_d

		# Check the status
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status != rb.run or exp_status != exp.wait:
			self.my_log.logerr("Navi: Cannot move now!")
			return False
		self.exp_s.Move() 							# (Important!) Change the status

		[qx, qy] = q # destination

		# Get the Robot's position
		try:
			[px, py] = self.trans.get_posi() # origin position
		except rospy.exceptions.ROSException:
			self.my_log.logerr("Navi: Cannot get the Robot's position!")
			self.exp_s.Wait() 						# (Important!) Change the status
			return False

		# Find path
		path = self.g.find_path(px, py, qx, qy)
		if len(path) == 0:
			self.my_log.logerr("Navi: Cannot find the path!")
			self.exp_s.Wait() 						# (Important!) Change the status
			return False
		self.my_log.loginfo("Navi: Found the path!")

		# Send path message
		path_msg = []
		path_msg.append(point_2d(px, py)) # origin
		for p in path:
			path_msg.append(point_2d(p[0], p[1]))
		self.path_pub.publish(path_msg)

		# Move to the destination
		self.my_log.loginfo("Navi: Start moving!")
		is_finished = self.move(path, eps_d)
		self.exp_s.Wait() 								# (Important!) Change the status
		if is_finished:
			self.my_log.loginfo("Navi: Arrive at destination!")
		else:
			self.my_log.logerr("Navi: Failed to achieve destination!")
		return is_finished

	def navi_move_along(self, ori_path, eps_d = -1):
		"Navi to move along the path: correct them and move along this path. Finally, send the corrected path to Control port"
		'''
		parameters:
			ori_path: the purposal path.
			eps_d: the tollerance of the destination. Default is -1. If eps_d <= 0, then it will be set as accu_eps_d.
		msgs:
			path:
				float64 start_time
				float64 stop_time
				point_2d[] p
			point_2d:
				float64 x
				float64 y
		'''
		if eps_d <= 0:
			eps_d = accu_eps_d

		# Check the status
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status != rb.run or exp_status != exp.wait:
			self.my_log.logerr("Navi: Cannot move now!")
			return False
		self.exp_s.Move() 							# (Important!) Change the status

		# Get the Robot's position
		try:
			[px, py] = self.trans.get_posi() # origin position
		except rospy.exceptions.ROSException:
			self.my_log.logerr("Navi: Cannot get the Robot's position!")
			self.exp_s.Wait() 							# (Important!) Change the status
			return False
		
		if not self.g.is_empt_coordinate(px, py):
			[tx, ty] = self.g.correct_point_coordinate(px, py) # correct the origin
			self.my_log.logwarn("Navi: The origin is illegal!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(px, py, tx, ty))
			px = tx
			py = ty
		
		path = [[px, py]]# add the origin point
		for p in ori_path:
			# Find path
			path_tmp = self.g.find_path(path[-1][0], path[-1][1], p[0], p[1])
			if len(path_tmp) == 0:
				self.my_log.logerr("Navi: Cannot find the path!")
				self.exp_s.Wait() 						# (Important!) Change the status
				return False
			path += path_tmp
		self.my_log.loginfo("Navi: Finished correcting the path!")

		# Send path message
		path_msg = []
		for p in path:
			path_msg.append(point_2d(p[0], p[1]))
		self.path_pub.publish(path_msg)

		# Move to the destination
		self.my_log.loginfo("Navi: Start moving!")
		is_finished = self.move(path, eps_d)
		self.exp_s.Wait() 								# (Important!) Change the status
		if is_finished:
			self.my_log.loginfo("Navi: Arrive at destination!")
		else:
			self.my_log.logerr("Navi: Failed to achieve destination!")
		return is_finished

	def navi_cmd(self, cmd):
		"Check the command and excute the command."
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status == rb.run and cmd in v_cmd["stop"]:
			self.exp_s.Wait()
			return
		if cmd in v_cmd["come here"]:
			if rb_status != rb.run or exp_status != exp.wait:
				self.my_log.logerr("Navi: Cannot move now!")
				self.response("Cannot move now!", False)
				return
			is_finished = self.move_to_user()
			if is_finished:
				self.my_log.loginfo("Navi: Move to user successfully!")
				self.response("", True)
			else:
				self.my_log.logerr("Navi: Failed to move to user!")
				self.response("Failed to move to user!", False)
		elif cmd in v_cmd["look me"]:
			if rb_status != rb.run or exp_status != exp.wait:
				self.my_log.logerr("Navi: Cannot move now!")
				self.response("Cannot move now!", False)
				return
			is_finished = self.face_to_user()
			if is_finished:
				self.my_log.loginfo("Navi: Face to user successfully!")
				self.response("", True)
			else:
				self.my_log.logerr("Navi: Failed to face to user!")
				self.response("Failed to face to user!", False)
		elif cmd in v_cmd["move to origin"]:
			if rb_status != rb.run or exp_status != exp.wait:
				self.my_log.logerr("Navi: Cannot move now!")
				self.response("Cannot move now!", False)
				return
			is_finished = self.move_to_origin_point()
			if is_finished:
				self.my_log.loginfo("Navi: Move to origin point successfully!")
				self.response("", True)
			else:
				self.my_log.logerr("Navi: Failed to move to origin point!")
				self.response("Failed to move to origin point!", False)
		elif cmd in v_cmd["move to the door"]:
			if rb_status != rb.run or exp_status != exp.wait:
				self.my_log.logerr("Navi: Cannot move now!")
				self.response("Cannot move now!", False)
				return
			is_finished = self.move_to_the_door()
			if is_finished:
				self.my_log.loginfo("Navi: Move to the door successfully!")
				self.response("", True)
			else:
				self.my_log.logerr("Navi: Failed to move to the door!")
				self.response("Failed to move to the door!", False)

	def pub_move_cmd(self, linear, angular):
		'''
		Normalize the velocities and publish them.
		The linear velocity and its difference is bounded by max_l and max_delta_l.
		The angular velocity and its difference is bounded by max_a and max_delta_a.
		'''
		linear = max(min(linear, max_l), -max_l)
		angular = max(min(angular, max_a), -max_a)
		delta_l = linear - self.move_cmd.linear.x
		delta_l = max(min(delta_l, max_delta_l), -max_delta_l)
		delta_a = angular - self.move_cmd.angular.z
		delta_a = max(min(delta_a, max_delta_a), -max_delta_a)
		self.move_cmd.linear.x += delta_l
		self.move_cmd.angular.z += delta_a
		self.move_cmd_pub.publish(self.move_cmd)

	def get_orientation(self, p1, p2):
		return atan2(p2[1] - p1[1], p2[0] - p1[0])

	def dist(self, a, b):
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def move(self, path, eps_d = -1):
		'''
		Move along this path. (No navigation)
		path: the path that robot will movw along.
		eps_d: the tollerance at the last point. Default is -1. If eps_d <= 0, then it will be set as accu_eps_d.
		'''
		if eps_d <= 0:
			eps_d = accu_eps_d
		l = len(path)
		for i in range(l):
			if self.exp_s.get_status() != exp.move:
				return False
			if i < l - 1:
				if not self.move_to(path[i]):
					return False
			else:
				if not self.move_to(path[i], eps_d = eps_d):
					return False
		return True

	def face_to(self, p, eps_a = -1):
		'''
		Face to the point p.
		eps_s: the tollerance of the angle. Default is -1. If eps_a <= 0, then it will be set as rough_eps_a.
		'''
		if eps_a <= 0:
			eps_a = rough_eps_a
		rate = rospy.Rate(10)
		print "Navi: Rotate!"
		while True:
			if self.exp_s.get_status() != exp.move and self.rb_s.get_status() != rb.init:
				return False

			try:
				msg = self.trans.get_msg()
			except rospy.exceptions.ROSException as e:
				self.my_log.logerr("Navi: Cannot get the position!\nDetails: %s", e)
				return False

			cur_p = self.trans.get_posi(msg)
			angle = self.trans.normalize_angle(self.get_orientation(cur_p, p) - self.trans.get_angle(msg))
			
			if abs(angle) < eps_a: 		# if the angle is tollerant
				self.pub_move_cmd(0, 0) # stop moving
				break 					# and break the loop
			self.pub_move_cmd(0, 2 * angle)
			rate.sleep()
		return True

	def move_to(self, p, eps_d = -1):
		'''
		Move to point p. (No navigation)
		eps_d: the tollerance of the point. Default is -1. If eps_d <= 0, then it will be set as rough_eps_d.
		'''
		if eps_d <= 0:
			eps_d = rough_eps_d
		rate = rospy.Rate(10)
		linear = 0
		angular = 0
		self.move_cmd = Twist()

		try:
			msg = self.trans.get_msg()
		except rospy.exceptions.ROSException as e:
			self.my_log.logerr("Navi: Cannot get the position!\nDetails: %s", e)
			return False

		d = self.dist(p, self.trans.get_posi(msg))
		if d < eps_d:
			return True
		
		print "Navi: Move to %s, %s"%(p[0], p[1])
		if not self.face_to(p):
			return False
		
		print "Navi: Move!"

		while True:
			if self.exp_s.get_status() != exp.move and self.rb_s.get_status() != rb.init:
				return False
			try:
				msg = self.trans.get_msg()
			except rospy.exceptions.ROSException as e:
				self.my_log.logerr("Navi: Cannot get the position!\nDetails: %s", e)
				return False
			
			# Compute the distance to p and delta angle pointing to p
			cur_p = self.trans.get_posi(msg)
			d = self.dist(p, cur_p)
			angle = self.trans.normalize_angle(self.get_orientation(cur_p, p) - self.trans.get_angle(msg))
			
			if d < eps_d:	# if the distance is tollerant
				break 		# break the loop
			
			# Compute the velocity
			if d*abs(angle) > eps_d: 	# if the angle is too large
				angular = 2 * angle  	# adjust the current angle to face p
			else:
				angular = 0
			angular = min(max_a, max(-max_a, angular))
			
			if abs(angle) > 1.0:		# if the angle is too large
				linear = 0				# decrease the linear velocity
			elif abs(angle) > 0.5:
				linear = d / 200
			else:
				linear = d / 100
			
			self.pub_move_cmd(linear, angular)
			rate.sleep()

		print "Navi: Arrive at %s, %s"%(cur_p[0], cur_p[1])

		return True

	def get_user_posi(self):
		p_user = rospy.get_param("user_posi")
		p_user[0] = int(p_user[0])
		p_user[1] = int(p_user[1])
		return p_user

	def is_proper_to_recog(self, x, y):
		d = self.dist([x, y], self.p_user)
		return d_user_min <= d and d <= d_user_max

	def get_std_recog_posi(self):
		[x, y] = self.get_user_posi()
		x = max(d_user_max, min(self.g.w - d_user_max, x))
		y = max(d_user_max, min(self.g.h - d_user_max, y))
		return [x, y]

	def face_to_user(self):
		# Check the status
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status != rb.run or exp_status != exp.wait:
			return False
		self.exp_s.Move() 							# (Important!) Change the status
		
		p_user = self.get_user_posi()
		is_finished = self.face_to(p_user, accu_eps_a)
		self.exp_s.Wait() 							# (Important!) Change the status
		return is_finished

	def move_to_user(self):
		# Check the status
		rb_status = self.rb_s.get_status()
		exp_status = self.exp_s.get_status()
		if rb_status != rb.run or exp_status != exp.wait:
			return
		self.exp_s.Move() 							# (Important!) Change the status

		p_user = self.get_user_posi()

		# Get the Robot's position
		try:
			[px, py] = self.trans.get_posi() # origin position
		except rospy.exceptions.ROSException:
			self.my_log.logerr("Navi: Cannot get the Robot's position!")
			px = 20
			py = 20
			#self.exp_s.Wait() 						# (Important!) Change the status
			#return False

		if self.is_proper_to_recog(px, py):
			is_finished = self.face_to(p_user, accu_eps_a)
			self.exp_s.Wait() 						# (Important!) Change the status
			return is_finished

		dst = self.get_std_recog_posi()

		# Find path
		_path = self.g.find_path(px, py, dst[0], dst[1])
		if len(_path) == 0:
			is_finished = self.face_to(p_user, accu_eps_a)
			self.exp_s.Wait() 						# (Important!) Change the status
			return is_finished
		
		# Modify the path while the distance is too large.
		if self.dist(p_user, [px, py]) > d_user_max:
			for i in range(len(_path)):
				if self.is_proper_to_recog(_path[i][0], _path[i][1]):
					path = _path[0:i + 1]
					break
			if  self.dist(path[-1], p_user) < d_user_min:
				p1 = path[-1]
				if len(path) > 1:
					p2 = path[-2]
				else:
					p2 = [px, py]
				lam = max((d_user_max - d_user_min) / self.dist(p1, p2), 1)
				path[-1][0] = lam * p1[0] + (1 - lam) * p2[0]
				path[-1][1] = lam * p1[1] + (1 - lam) * p2[1]
		self.my_log.loginfo("Navi: Found the path to the user!")
		
		# Send path message
		path_msg = []
		path_msg.append(point_2d(px, py)) # origin
		for p in path:
			path_msg.append(point_2d(p[0], p[1]))
		self.path_pub.publish(path_msg)

		self.my_log.loginfo("Navi: Move to %s!", path[-1])
		# Move to the destination
		self.my_log.loginfo("Navi: Start moving!")
		if not self.move(path):
			self.exp_s.Wait() 						# (Important!) Change the status
			return False
		if not self.face_to(p_user):
			self.exp_s.Wait() 						# (Important!) Change the status
			return False
		self.exp_s.Wait() 							# (Important!) Change the status
		return True

	def move_to_origin_point(self):
		if self.navi_move_to([0, 0]):
			self.exp_s.Move() 						# (Important!) Change the status# Get the Robot's position
			try:
				[px, py] = self.trans.get_posi() # origin position
			except rospy.exceptions.ROSException:
				self.my_log.logerr("Navi: Cannot get the Robot's position!")
				self.exp_s.Wait() 						# (Important!) Change the status
				return False
			is_finished = self.face_to([100, py], accu_eps_a)
			self.exp_s.Wait() 						# (Important!) Change the status
			return is_finished
		else:
			return False

	def move_to_the_door(self):
		return self.navi_move_to([self.g.w, self.g.h])


if __name__ == '__main__':
	try:
		navi = navi_nodes()
		navi.start()
	except rospy.ROSInterruptException:
		pass
