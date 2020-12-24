#!/usr/bin/env python
# -*- coding: utf-8 -*-

from robot_port.log import log
from robot_port.enum_list import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
from robot_port.msg import voice_cmd
from roslib import message

'''
sensor_msgs/CameraInfo.msg:
	std_msgs/Header header
	uint32 height
	uint32 width
	string distortion_model
	float64[] D
	float64[9] K
	float64[9] R
	float64[12] P
		[fx'  0  cx' Tx]
	P = [ 0  fy' cy' Ty]
		[ 0   0   1   0]
	uint32 binning_x
	uint32 binning_y
	sensor_msgs/RegionOfInterest roi

sensor_msgs/Image.msg:
	std_msgs/Header header
	uint32 height
	uint32 width
	string encoding
	uint8 is_bigendian
	uint32 step
	uint8[] data
'''

BAD_POINT = -1e10
SCALE = 0.001
DELTA = 1e-5
MAX_DIST = 4.0
r_search = 10

try:
	import cv2 as cv
	from cv_bridge import CvBridge, CvBridgeError
	bridge = CvBridge()
except ImportError as e:
	print e

def _to_meter_(x):
	return SCALE * x

def is_valid_depth(depth):
	return depth > 0.1 and depth < MAX_DIST


def _loop_index_ard(i, j, i_range, j_range, N):
	index = [[i, j]]
	for l in range(1, N):
		for k in range(l):
			index.append([i + k, j + l - k])
			index.append([i + l - k, j - k])
			index.append([i - k, j - l + k])
			index.append([i - l + k, j + k])
	return index

def read_depth(depth_image, px, py):
	px = int(px)
	py = int(py)
	assert depth_image is not None, "No depth image!"
	if (px >= depth_image.width) or (px < 0) or (py >= depth_image.height) or (py < 0):
		return -1
	try:
		cv_image = bridge.imgmsg_to_cv2(depth_image, depth_image.encoding)
	except Exception as e:
		print e
	index = _loop_index_ard(py, px, depth_image.height, depth_image.width, r_search)
	cnt = 0
	depth_valid_total = 0
	for p in index:
		depth = _to_meter_(cv_image[p[0]][p[1]])
		if is_valid_depth(depth):
			cnt += 1
			depth_valid_total += depth
	if cnt == 0:
		depth = 0
	else:
		depth = depth_valid_total / cnt
	return depth
	

class trans_of_camera:
	"Transfer between pixel and 3D point."
	def __init__(self):
		self.fx = 0
		self.fy = 0
		self.cx = 0
		self.cy = 0
		self.tx = 0
		self.ty = 0
		self.hx = 0
		self.hy = 0
		self.is_set = False     # represent whether camera_info is set

	def set_camera_info(self, camera_info):
		self.fx = camera_info.P[0]
		self.fy = camera_info.P[5]
		self.cx = camera_info.P[2]
		self.cy = camera_info.P[6]
		self.tx = camera_info.P[3]
		self.ty = camera_info.P[7]
		if self.fx < DELTA or self.fy < DELTA:
			return False
		self.hx = self.tx / self.fx
		self.hy = self.ty / self.fy
		self.is_set = True
		return True

	def pixel_to_3d(self, px, py, depth):
		if not self.is_set: return []
		if not is_valid_depth(depth):
			return [BAD_POINT, BAD_POINT, BAD_POINT]
		x = depth * (px - self.cx) / self.fx - self.hx
		y = depth * (py - self.cy) / self.fy - self.hy
		z = depth
		return [x, y, z]

	def point3d_to_pixel(self, x, y, z):
		if not self.is_set: return []
		if not is_valid_depth(z):
			return [-1, -1]
		px = self.fx * x / z + self.cx + self.tx / z
		py = self.fy * y / z + self.cy + self.ty / z
		return [px, py]


class convert:
	def __init__(self):
		self.tr_camera = trans_of_camera()
		self.depth_image = None

	def set_depth_image(self, depth_image):
		self.depth_image = depth_image

	def set_camera_info(self, camera_info):
		self.tr_camera.set_camera_info(camera_info)

	def is_set(self):
		"Return whether camera_info is set"
		return self.tr_camera.is_set

	def get_depth(self, px, py, depth_image = None):
		if not self.is_set(): return
		px = int(px)
		py = int(py)
		if depth_image is not None:
			self.set_depth_image(depth_image)
		return read_depth(self.depth_image, px, py)

	def get_3d_point(self, px, py, depth_image = None):
		if not self.is_set(): return []
		try:
			depth = self.get_depth(px, py, depth_image)
			print "depth of %s is %s"%([px, py], depth)
		except AssertionError as e:
			print e
			return []
		return self.tr_camera.pixel_to_3d(px, py, depth)

	def convert(self, depth_image):
		return
		
