#!/usr/bin/env python


import thread
import rospy
import socket
from robot_port.status import rb, exp
from robot_port.log import log
from robot_port.enum_list import *
from std_msgs.msg import String
from robot_port.msg import posi
from robot_port.msg import path_ori
from robot_port.msg import path
from robot_port.msg import voice_cmd
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import enum_type
from robot_port.msg import stop

import rb_message.robot_server as msg_server
import rb_message.robot_client as msg_client

def get_host_ip():
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect(('8.8.8.8', 80))
		ip = s.getsockname()[0]
	finally:
		s.close()
	return ip

class communicater:

	def __init__(self):
		rospy.init_node('communicater', anonymous = False)

		# Members
		self.rb_s = rb()
		self.exp_s = exp()
		self.my_log = log()

		#Publisher and Subscriber
		self.map_pub = rospy.Publisher('virtual_map', vmap, queue_size = 5)
		self.drive_pub = rospy.Publisher('drive_cmd', enum_type, queue_size = 10)
		self.stop_pub = rospy.Publisher('stop_exp', stop, queue_size = 2)

		# TEST_MODE and set the IP and Port of other port
		global TEST_MODE
		TEST_MODE = bool(rospy.get_param("TEST_MODE"))
		if not TEST_MODE:
			try:
				self.set_IP()
			except Exception as e:
				self.my_log.logerr("Cannot set the IP and Port of other ports!\nDetails: %s", e)
		
		rospy.Subscriber('posi_pub', posi, self.send_rb_posi)
		rospy.Subscriber('path_ori', path_ori, self.send_rb_path_ori)
		rospy.Subscriber('path', path, self.send_rb_path)
		rospy.Subscriber('voice_cmd', voice_cmd, self.send_rb_voice_cmd)
		rospy.Subscriber('response_to_ctrl', enum_type, self.send_response_to_ctrl)
		rospy.Subscriber('log_msg', String, self.send_log_msg)

		# Start the msg server
		receive_srv = msg_server.RobotServicer(self.receive_map, self.receive_command, self.receive_voice, self.receive_drive_command)
		try:
			thread.start_new_thread(msg_server.serve, (receive_srv, ))
			self.my_log.loginfo("Communicater: Receiving service Initialized!")
		except:
			self.my_log.logerr("Communicater: Unable to start the receiving service thread!")
			self.init_ok = False
			return

		self.my_log.loginfo("Communicater: Communicate Node Initialized!")
		self.init_ok = True
		return

	def input_IP(self):
		IP = str(raw_input("Please input the IP of Control Port:"))
		if IP != "":
			msg_client.receiverAddr["Ctrl"]["IP"] = IP
		port = int(raw_input("Please input the Port of Control Port:"))
		if port != "":
			msg_client.receiverAddr["Ctrl"]["Port"] = port
		IP = str(raw_input("Please input the IP of AR Port:"))
		if IP != "":
			msg_client.receiverAddr["AR"]["IP"] = IP
		port = int(raw_input("Please input the Port of AR Port:"))
		if port != "":
			msg_client.receiverAddr["AR"]["Port"] = port

	def read_IP(self, path):
		with open(path, "r") as f:
			settings = f.readlines()
			if len(settings) < 4:
				self.write_IP(path)
				return
			msg_client.receiverAddr["Ctrl"]["IP"] = settings[0][:-1]
			msg_client.receiverAddr["Ctrl"]["Port"] = settings[1][:-1]
			msg_client.receiverAddr["AR"]["IP"] = settings[2][:-1]
			msg_client.receiverAddr["AR"]["Port"] = settings[3]

	def write_IP(self, path):
		with open(path, "w") as f:
			f.write(msg_client.receiverAddr["Ctrl"]["IP"] + "\n")
			f.write(str(msg_client.receiverAddr["Ctrl"]["Port"]) + "\n")
			f.write(msg_client.receiverAddr["AR"]["IP"] + "\n")
			f.write(str(msg_client.receiverAddr["AR"]["Port"]))

	def set_IP(self):
		import os
		path = str(os.path.dirname(os.path.realpath(__file__)) + "/IP.txt")
		IP = get_host_ip()
		msg_client.receiverAddr["Robot"]["IP"] = IP
		print "The IP of Robot Port is:", IP
		print "The Port of Robot Port is:", msg_client.receiverAddr["Robot"]["Port"]
		try:
			self.read_IP(path)
		except IOError:
			self.write_IP(path)
		print "The IP of Control Port is:", msg_client.receiverAddr["Ctrl"]["IP"]
		print "The Port of Control Port is:", msg_client.receiverAddr["Ctrl"]["Port"]
		print "The IP of AR Port is:", msg_client.receiverAddr["AR"]["IP"]
		print "The Port of AR Port is:", msg_client.receiverAddr["AR"]["Port"]
		if str(raw_input("Do you want to change it? [y/n]:")) in ["y", "yes", "Yes", "YES"]:
			self.input_IP()
			self.write_IP(path)

	def start(self):
		rospy.spin()

	def send_rb_posi(self, msg):
		'''
		posi:
			float64 stamp
			float64 x
			float64 y
			float64 vx
			float64 vy
			float64 angle
		'''
		connected = bool(rospy.get_param("connected"))
		if TEST_MODE or not connected:
			return
		try:
			msg_client.sendRBPosition("Ctrl", [msg.x, msg.y], msg.angle, [msg.vx, msg.vy], msg.stamp)
			# print rospy.Time.now().to_sec() - msg.stamp
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the position! Cannot connect to Control port! Details:\n %s", e)
			rospy.set_param("connected", False)
		try:
			msg_client.sendRBPosition("AR", [msg.x, msg.y], msg.angle, [msg.vx, msg.vy], msg.stamp)
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the position! Cannot connect to AR port! Details:\n %s", e)
			rospy.set_param("connected", False)
		return

	def send_rb_path_ori(self, msg):
		'''
		path_ori:
			float64 start_time
			float64 end_time
			point_2d[] p
		point_2d:
			float64 x
			float64 y
		'''
		connected = bool(rospy.get_param("connected"))
		if TEST_MODE or not connected:
			return
		path = []
		for p in msg.p:
			path.append([p.x, p.y])
		try:
			msg_client.sendRBPath("Ctrl", path, msg.start_time, msg.end_time)
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the original path! Cannot connect to Control port! Details:\n %s", e)
			rospy.set_param("connected", False)
		return

	def send_rb_path(self, msg):
		'''
		path:
			point_2d[] p
		point_2d:
			float64 x
			float64 y
		'''
		connected = bool(rospy.get_param("connected"))
		if TEST_MODE or not connected:
			return
		if len(msg.p) == 0:
			return
		path = []
		for p in msg.p:
			path.append([p.x, p.y])
		try:
			msg_client.sendRBPath("Ctrl", path)
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the path! Cannot connect to Control port!\nDetails: %s", e)
			rospy.set_param("connected", False)
		return

	def send_rb_voice_cmd(self, msg):
		'''
		voice_cmd:
			float64 stamp
			string cmd
		'''
		connected = bool(rospy.get_param("connected"))
		if TEST_MODE or not connected:
			return
		try:
			msg_client.sendVoiceResult("Ctrl", msg.cmd, msg.stamp)
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the voice cmd! Cannot connect to Control port!\nDetails: %s", e)
			rospy.set_param("connected", False)
		try:
			msg_client.sendVoiceResult("AR", msg.cmd, msg.stamp)
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the voice cmd! Cannot connect to AR port!\nDetails: %s", e)
			rospy.set_param("connected", False)
		return

	def send_response_to_ctrl(self, msg):
		'''
		voice_cmd:
			float64 stamp
			string cmd
		'''
		connected = bool(rospy.get_param("connected"))
		if TEST_MODE or not connected:
			return
		try:
			msg_client.sendResponseMsg("Ctrl", msg.type)
		except Exception as e:
			self.my_log.logerr("Communicater: Failed to send the response! Cannot connect to Control port!\nDetails: %s", e)
			rospy.set_param("connected", False)
		return

	def send_log_msg(self, msg):
		'''
		voice_cmd:
			float64 stamp
			string cmd
		'''
		connected = bool(rospy.get_param("connected"))
		if TEST_MODE or not connected:
			return
		try:
			msg_client.sendLogMsg("Ctrl", msg.data)
		except Exception as e:
			rospy.logerr("Communicater: Failed to send the log msg! Cannot connect to Control port!\nDetails: %s", e)
			rospy.set_param("connected", False)
		return

	def receive_map(self, request, context):
		"Handling message 'map' from control_port"
		'''
		map_object:
			bool type   # False means CUBE, True means CYLINDER
			float64 x
			float64 y
			float64 w
			float64 h
		vmap:
			float32 w
			float32 h
			map_object[] obj
		'''
		status = self.rb_s.get_status()
		if status == rb.sleep:
			self.my_log.logerr("Communicater: Cannot init the map. The Exp hasn't started yet!")
			return 0
		elif status != rb.init:
			self.my_log.logerr("Communicater: Cannot init the map. There is an Exp running now!")
			return 0
		obj = []
		for block in request.blocks:
			obj.append(map_object(block.type, block.pos.posx, block.pos.posy, block.w, block.h))
		self.map_pub.publish(request.roomwidth, request.roomheight, obj) # Pubilsh the map to topic 'virtual_map'
		return 0

	def receive_command(self, request, context):
		status = self.rb_s.get_status()
		cmd = int(request.cmd)
		if cmd == START: # start the exp
			if status == rb.sleep:
				self.rb_s.Start()
				self.my_log.loginfo("Communicater: Start initializing the Exp.")
			else:
				self.my_log.logerr("Communicater: Cannot start the Exp. There is an Exp running now!")
		elif cmd == STOP: # stop the exp
			if status != rb.sleep:
				self.exp_s.Wait()
				self.rb_s.Stop()
				self.stop_pub.publish(True)
				self.my_log.loginfo("Communicater: Stop the Exp successfully!")
			else:
				self.my_log.logwarn("Communicater: Cannot stop the Exp. The Exp hasn't started yet!")
		elif cmd == CONNECT:
			rospy.set_param("connected", True)
			self.my_log.loginfo("Build connect") # build connect
		else:
			return 1
		return 0

	def receive_drive_command(self, request, context):
		cmd = request.drivecmd
		if self.rb_s.get_status() == rb.run and self.exp_s.get_status() == exp.move:
			return
		self.drive_pub.publish(cmd)
		return 0

	def receive_voice(self, request_iterator, context):
		return 0

		


if __name__ == '__main__':
	try:
		comm = communicater()
		if comm.init_ok:
			comm.start()
	except rospy.ROSInterruptException:
		msg_server.stop()
