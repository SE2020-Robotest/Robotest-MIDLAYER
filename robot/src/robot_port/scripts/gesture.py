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
from robot_port.msg import point_2d
from robot_port.msg import response
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
		with open(self.path + name + ".data", "wb") as f:
			f.write(cl_img)

	def read(self, name):
		with open(self.path + name + ".data", "wb") as f:
			data = pockle.loads(f)
			return data

	def save_camera_info(self, depth_camera, color_camera):
		self.save(depth_camera, "/depth_camera")
		self.save(color_camera, "/color_camera")

	def save_recognition_data(self, depth_image, color_image, frame):
		self.num += 1
		self.save(depth_image, "/%s/depth_image"%self.num)
		self.save(color_image, "/%s/color_image"%self.num)
		self.save(frame, "/%s/frame"%self.num)

	def read_camera_info(self):
		depth_camera = CameraInfo(self.read("/depth_camera"))
		color_camera = CameraInfo(self.read("/color_camera"))
		return (depth_camera, color_camera)

	def read_recognition_data(self, num = 1):
		try:
			depth_image = Image(self.read("/%s/depth_image"%num))
			color_image = Image(self.read("/%s/color_image"%num))
			frame = Frame(self.read("/%s/frame"%num))
		except IOError as e:
			print e
			return
		return (depth_image, color_image, frame)

class gesture_recognize:
	def __init__(self, my_log = None):
		self.convert = convert.convert()
		self.radio_x = 0
		self.radio_y = 0
		self.tr = trans()
		self.camera_frame_id = ""
		self.transformation = None
		self.recognizing = False
		self.my_log = my_log

	def start_recognizing(self, depth_image):
		if self.is_set() and depth_image is not None and (not self.recognizing):
			self.convert.set_depth_image(depth_image)
			self.transformation = self.tr.get_transformation(self.camera_frame_id, VMAP_FRAME, depth_image.header.stamp)
			self.recognizing = True
			return True
		else:
			return False

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
		return self.tr.point_a_to_b(p_camera[0], p_camera[1], p_camera[2], self.transformation)

	def compute_dst(self, p, q):
		assert abs(p[2] - q[2]) > 0.0001, "Cannot compute the intersection properly!"
		t = p[2] / (p[2] - q[2])
		dst[0] = (1 - t) * p[0] + t * q[0]
		dst[1] = (1 - t) * p[1] + t * q[1]
		return dst

	def recognize(self, person):
		if not self.recognizing:
			return []
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
			self.recognizing = False
			return []
		try:
			dst = self.compute_dst(p_right_shoulder, p_right_hand)
		except AssertionError as e:
			if self.my_log is None:
				print "Gesture: Error: %s"%e
			else:
				self.my_log.logerr("Gesture: %s", e)
			self.recognizing = False
			return []
		self.recognizing = False
		return [p_right_shoulder, p_right_hand, dst]

class Gesture:
	def __init__(self):
		rospy.init_node('gesture', anonymous = False)

		self.my_log = log()

		rospy.Subscriber('voice_cmd', voice_cmd, self.receive_voice)
		self.dst_pub = rospy.Publisher('dst', point_2d, queue_size = 1)
		self.response_pub = rospy.Publisher('response', response, queue_size = 10)

		global TEST_MODE
		TEST_MODE = bool(rospy.get_param("TEST_MODE"))
		if TEST_MODE:
			path = rospy.get_param("pkg_path") + "/data/test"
			self.save_data = recognition_data(path)
			(depth_camera, color_camera) = self.save_data.read_camera_info()
			self.receive_depth_camera_info(depth_camera)
			self.receive_color_camera_info(color_camera)
			(depth_image, color_image, frame) = self.save_data.read_recognition_data()
			self.receive_depth_image(depth_image)
			self.receive_color_image(color_image)
			self.test_frame = frame
		else:
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
			self.gesture_recognizer = gesture_recognize()
			self.color_image_pub = rospy.Publisher('/camera/color/image_raw/my_pub', Image, queue_size = 1)
			self.depth_image_pub = rospy.Publisher('/camera/depth/image_raw/my_pub', Image, queue_size = 1)
			rospy.Subscriber('/camera/color/image_raw', Image, self.receive_color_image)
			rospy.Subscriber('/camera/depth/image_raw', Image, self.receive_depth_image)
			self.dp_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.receive_depth_camera_info)
			self.cl_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.receive_color_camera_info)
			rospy.Subscriber('/frame', Frame, self.receive_frame)

		self.exp = exp()

		self.is_color_cemera_set = False
		self.is_depth_cemera_set = False
		self.recognizing = False
		self.color_image = None
		self.depth_image = None
		self.depth_point_cloud = None
		
		self.my_log.loginfo("Gesture: Gesture Node Initialized!")

	def start(self):
		rospy.spin()

	def response(self, discription, response):
		self.response_pub.publish(rospy.Time.now().to_sec(), "Gesture", discription, response)

	def is_set(self):
		return self.is_depth_cemera_set and self.is_color_cemera_set

	def finish_set(self):
		radio_x = float(self.depth_camera_info.width) / float(self.color_camera_info.width)
		radio_y = float(self.depth_camera_info.height) / float(self.color_camera_info.height)
		self.gesture_recognizer.set_radio(radio_x, radio_y)
		self.save_data.save_camera_info(self.depth_camera_info, self.color_camera_info)
		self.dp_info_sub.unregister(CameraInfo)
		self.cl_info_sub.unregister(CameraInfo)
		del self.depth_camera_info
		del self.color_camera_info
		del self.dp_info_sub
		del self.cl_info_sub

	def receive_voice(self, msg):
		cmd = msg.cmd
		if cmd in v_cmd["move to there"]:
			if not self.exp.Recog():
				self.my_log.logerr("Gesture: Cannot recognize now!")
				return
			if self.image_pub() and self.gesture_recognizer.start_recognizing(self.depth_image):
				self.my_log.loginfo("Gesture: Published the image")
			else:
				self.exp.Wait()
				if TEST_MODE:
					rospy.sleep(5)
					self.receive_frame(self.frame)

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
			self.response("No person detected!", False)
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
			self.exp.Wait()
			self.dst_pub.publish(dst[0], dst[1])
			self.response("", True)
		elif 0 < len(p):
			self.my_log.logerr("Gesture: The number of feedback is wrong!")
			self.exp.Wait()
			self.response("The number of feedback is wrong!", False)
		return

	def image_pub(self):
		if self.color_image is None or self.color_image is None:
			self.my_log.logerr("Gesture: No image received!")
			return False
		if TEST_MODE:
			return True
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

