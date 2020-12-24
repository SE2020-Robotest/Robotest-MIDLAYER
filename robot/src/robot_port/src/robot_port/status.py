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
	recog = "recognizing"
	move = "moving"

	def __init__(self):
		return

	def get_status(self):
		return get_s(exps)

	def set_status(self, st):
		set_s(exps, st)

	def is_running(self):
		return get_s(rbs) == rb.run

	def Recog(self):
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

def test_status():
	failed_str = ["Initial status is wrong!", "An error occurred while running rb.Start", "An error occurred while running rb.Run", \
					"An error occurred while running rb.Stop", "An error occurred while running exp.Wait", "An error occurred while running exp.Move", \
					"An error occurred while running exp.Recog", "An error occurred in status transition"]
	rb_s = rb()
	exp_s = exp()
	assert rb_s.get_status() == rb.sleep, failed_str[0]
	assert exp_s.get_status() == exp.wait, failed_str[0]
	assert not exp_s.Wait(), failed_str[4]
	assert not exp_s.Move(), failed_str[5]
	assert not exp_s.Recog(), failed_str[6]
	assert rb_s.Start(), failed_str[1]
	assert rb_s.get_status() == rb.init, failed_str[7]
	assert rb_s.Stop(), failed_str[3]
	assert rb_s.Start(), failed_str[1]
	assert not exp_s.Wait(), failed_str[4]
	assert not exp_s.Move(), failed_str[5]
	assert not exp_s.Recog(), failed_str[6]
	assert rb_s.Run(), failed_str[2]
	assert rb_s.get_status() == rb.run, failed_str[7]
	assert exp_s.Wait(), failed_str[4]
	assert exp_s.get_status() == exp.wait, failed_str[7]
	assert exp_s.Move(), failed_str[5]
	assert exp_s.get_status() == exp.move, failed_str[7]
	assert not exp_s.Recog(), failed_str[6]
	assert exp_s.get_status() == exp.move, failed_str[7]
	assert exp_s.Wait(), failed_str[4]
	assert exp_s.Recog(), failed_str[6]
	assert exp_s.get_status() == exp.recog, failed_str[7]
	assert exp_s.Wait(), failed_str[4]
	assert rb_s.Stop(), failed_str[3]
	assert not rb_s.Run(), failed_str[2]
	

