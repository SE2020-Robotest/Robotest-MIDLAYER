#!/usr/bin/env python


from rospy import get_param as get_s
from rospy import set_param as set_s


rbs = "robot_status"
exps = "exp_status"

class rb:
	sleep = "sleeping"
	init = "initializing"
	run = "running"

	def __init__(self):
		return

	def get_status(self):
		return get_s(rbs)

	def set_status(self, st):
		set_s(rbs, st)

	def Start(self):
		if self.get_status() != self.sleep:
			return False
		self.set_status(self.init)
		return True

	def Run(self):
		if self.get_status() != self.init:
			return False
		self.set_status(self.run)
		return True

	def Stop(self):
		self.set_status(self.sleep)
		return True

class exp:
	wait = "waiting"
	recog = "recogniting"
	move = "moving"

	def __init__(self):
		return

	def get_status(self):
		return get_s(exps)

	def set_status(self, st):
		set_s(exps, st)

	def is_running(self):
		return get_s(rbs) == rb.run

	def Recognite(self):
		if not self.is_running() or self.get_status() != self.wait:
			return False
		self.set_status(self.recog)
		return True

	def Move(self):
		if not self.is_running() or self.get_status() != self.wait:
			return False
		self.set_status(self.move)
		return True

	def Wait(self):
		if not self.is_running():
			return False
		self.set_status(self.wait)
		return True
