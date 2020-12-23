#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pickle
import robot_port.convert as convert
from robot_port.log import log
from robot_port.enum_list import *
from robot_port.trans import trans
from robot_port.status import exp
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
from robot_port.msg import voice_cmd
from robot_port.msg import Frame
from roslib import message

'''
Frame:
	Header header
	Person[] persons
Person:
	BodyPart[] bodyParts
	BodyPart[] leftHandParts
	BodyPart[] rightHandParts
BodyPart:
	float32 score
	Pixel pixel
	geometry_msgs/Point32 point
Pixel:
	float32 x
	float32 y
'''

class recognition_data:
	def __init__(self, path = None):
		self.num = 0
		if path is None:
			import os
			self.path = os.path.dirname(os.path.realpath(__file__))
		else:
			self.path = path

	def save(self, data, name):
		raw_data = pickle.dumps(data)
		with open(self.path + "/%s/"%self.num + name + ".data", "wb") as f:
			f.write(cl_img)

	def save_recognition_data(self, depth_image, color_image, frame):
		self.num += 1
		self.save(depth_image, "depth_image")
		self.save(color_image, "color_image")
		self.save(frame, "frame")

	def read_recognition_data(self, num):
		
		return

class gesture_recognize:
	def __init__(self, my_log = None):
		self.convert = convert.convert()
		self.radio_x = 0
		self.radio_y = 0
		self.tr = trans()
		self.camera_frame_id = ""
		self.my_log = my_log

	def set_depth_image(self, depth_image):
		self.convert.set_depth_image(depth_image)

	def set_camera_info(self, camera_info):
		self.convert.set_camera_info(camera_info)
		self.camera_frame_id = camera_info.header.frame_id

	def set_radio(self, radio_x, radio_y):
		self.radio_x = radio_x
		self.radio_y = radio_y

	def is_set(self):
		"Return whether camera_info is set"
		return self.convert.is_set()

	def get_3d_of_body_part(self, body_part):
		return self.convert.get_3d_point(body_part.pixel.x * self.radio_x, body_part.pixel.y * self.radio_y)

	def log_body_part(self, body_part, name):
		px = body_part.pixel.x
		py = body_part.pixel.y
		p = self.get_3d_of_body_part(body_part)
		s = name + ": 2D pixel coordinates is %s, score is %s, 3D point coordinates is %s."%([px, py], body_part.score, p)
		if self.my_log is None:
			print s
		else:
			self.my_log.loginfo(s)
		return

	def log_key_point(self, person):
		right_shoulder = person.bodyParts[2]
		right_hand = person.bodyParts[4]
		self.log_body_part(right_shoulder, "Right Shoulder")
		self.log_body_part(right_hand, "Right hand")
		return

	def point_camera_to_vmap(self, p_camera):
		return self.tr.point_a_to_b(self.camera_frame_id, VMAP_FRAME, p_camera)

	def compute_dst(self, p, q):
		assert abs(p[2] - q[2]) > 0.0001, "Cannot compute the intersection properly!"
		t = p[2] / (p[2] - q[2])
		dst[0] = (1 - t) * p[0] + t * q[0]
		dst[1] = (1 - t) * p[1] + t * q[1]
		return dst

	def recognize(self, person):
		right_shoulder = person.bodyParts[2]
		right_hand = person.bodyParts[4]
		p_right_shoulder_camera = self.get_3d_of_body_part(right_shoulder)
		p_right_hand_camera = self.get_3d_of_body_part(right_hand)
		try:
			p_right_shoulder = self.point_camera_to_vmap(p_right_shoulder_camera)
			p_right_hand = self.point_camera_to_vmap(p_right_hand_camera)
		except Exception as e:
			if self.my_log is None:
				print "Gesture: Cannot transform the key point to vmap!\nDetails: %s"%e
			else:
				self.my_log.logerr("Gesture: Cannot transform the key point to vmap!\nDetails: %s", e)
			return []
		try:
			dst = self.compute_dst(p_right_shoulder, p_right_hand)
		except AssertionError as e:
			if self.my_log is None:
				print "Gesture: Error: %s"%e
			else:
				self.my_log.logerr("Gesture: %s", e)
			return []
		return [p_right_shoulder, p_right_hand, dst]

class Gesture:
	def __init__(self):
		rospy.init_node('gesture', anonymous = False)

		self.my_log = log()
		self.gesture_recognizer = gesture_recognize()
		self.exp = exp()

		import os
		if rospy.has_param("pkg_path"):
			path = rospy.get_param("pkg_path")
		else:
			path = os.path.dirname(os.path.realpath(__file__))
		path += "/data"
		if not os.path.exists(path):
			os.makedirs(path)
		path += "/" + rospy.get_param("start_time")
		if not os.path.exists(path):
			os.makedirs(path)
		self.save_data = recognition_data(path)

		self.is_color_cemera_set = False
		self.is_depth_cemera_set = False
		self.recognizing = False
		self.color_image = None
		self.depth_image = None
		self.depth_point_cloud = None

		self.color_image_pub = rospy.Publisher('/camera/color/image_raw/my_pub', Image, queue_size = 1)
		self.depth_image_pub = rospy.Publisher('/camera/depth/image_raw/my_pub', Image, queue_size = 1)
		rospy.Subscriber('voice_cmd', voice_cmd, self.receive_voice)
		rospy.Subscriber('/camera/color/image_raw', Image, self.receive_color_image)
		rospy.Subscriber('/camera/depth/image_raw', Image, self.receive_depth_image)
		rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.receive_depth_camera_info)
		rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.receive_color_camera_info)
		rospy.Subscriber('/frame', Frame, self.receive_frame)
		
		self.my_log.loginfo("Gesture: Gesture Node Initialized!")

	def start(self):
		rospy.spin()

	def is_set(self):
		return self.is_depth_cemera_set and self.is_color_cemera_set

	def finish_set(self):
		radio_x = float(self.depth_camera_info.width) / float(self.color_camera_info.width)
		radio_y = float(self.depth_camera_info.height) / float(self.color_camera_info.height)
		self.gesture_recognizer.set_radio(radio_x, radio_y)
		del self.depth_camera_info
		del self.color_camera_info

	def receive_voice(self, msg):
		cmd = msg.cmd
		if cmd in v_cmd["move to there"]:
			if not self.exp.Recog():
				self.my_log.logerr("Gesture: Cannot recognize now!")
				return
			if self.image_pub():
				self.gesture_recognizer.set_depth_image(self.depth_image)
				self.my_log.loginfo("Gesture: Published the image")
			else:
				self.exp.Wait()

	def receive_depth_camera_info(self, data):
		if not self.is_depth_cemera_set:
			self.gesture_recognizer.set_camera_info(data)
			self.depth_camera_info = data
			self.my_log.loginfo("Set the depth camera info successfully!")
			self.is_depth_cemera_set = True
			if self.is_set(): self.finish_set()

	def receive_color_camera_info(self, data):
		if not self.is_color_cemera_set:
			self.my_log.loginfo("Set the color camera info successfully!")
			self.color_camera_info = data
			self.is_color_cemera_set = True
			if self.is_set(): self.finish_set()

	def receive_color_image(self, data):
		if not self.recognizing:
			self.color_image = data

	def receive_depth_image(self, data):
		if not self.recognizing:
			self.depth_image = data

	def receive_frame(self, data):
		self.my_log.loginfo("Gesture: Got the frame")
		self.save_data.save_recognition_data(self.depth_image, self.color_image, data)
		persons = data.persons
		if len(persons) == 0:
			self.my_log.logerr("Gesture: No person detected!")
			self.recognizing = False
			return
		elif len(persons) > 1:
			self.my_log.logwarn("Gesture: More than one person detected!")
		person = persons[0]
		if not self.is_set():
			self.my_log.logerr("Gesture: The camera info has not been set yet!")
		self.gesture_recognizer.log_key_point(person)
		p = self.gesture_recognizer.recognize(person)
		if len(p) == 3:
			[p_right_shoulder, p_right_hand, dst] = p
			self.my_log.loginfo("The right shoulder is %s, right hand is %s. The destination is %s", p_right_shoulder, p_right_hand, dst)
		elif 0 < len(p):
			self.my_log.logerr("Gesture: The number of feedback is wrong!")
		self.exp.Wait()
		return

	def image_pub(self):
		if self.color_image is None or self.color_image is None:
			self.my_log.logerr("Gesture: No image received!")
			return False
		self.color_image_pub.publish(self.color_image)
		self.depth_image_pub.publish(self.depth_image)
		return True


"""
	def read_depth(self, width, height, data) :
		# read function
		if (height >= data.height) or (width >= data.width) :
			return -1
		data_out = point_cloud2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height]])
		int_data = next(data_out)
		self.my_log.loginfo("int_data " + str(int_data))
		return int_data

	def print_depth(self, data):
		print data.height, data.width, data.header.frame_id
		self.read_depth(400, 300, data)
"""



if __name__ == '__main__':
	try:
		r = Gesture()
		r.start()
	except rospy.ROSInterruptException:
		pass

