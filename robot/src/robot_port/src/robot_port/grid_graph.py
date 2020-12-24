#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
The origin point is on the bottom-left, and unit is cm

i           y
|           |
|        &  |
|_____ j    |_____ x

This is the corresbonding relation between grid_graph and virtual_map
'''

from rospy import logwarn
from enum import Enum
from math import sqrt
from my_pq import my_pq
from enum_list import *


class v_type(Enum):
	empt = 0
	full = 1

class grid_graph:

	def __init__(self, w = 100, h = 100, log = None):
		self.log = log
		self.w = w
		self.h = h
		self.i_range = int(h / delta)
		self.j_range = int(w / delta)
		self.N = self.j_range * self.i_range
		self.v = [[v_type.empt]*self.j_range for i in range(self.i_range)]
		self.finished = False;
		return

	def is_inside(self, i, j):
		"Determine whether i,j is in the graph"
		return 0 <= i and i < self.i_range and 0 <= j and j < self.j_range

	def is_empt(self, i, j):
		"Determine whether i,j is in the graph, and v[i][j] is empty"
		return self.is_inside(i, j) and self.v[i][j] == v_type.empt

	def is_empt_coordinate(self, x, y):
		j = int(x // delta)
		i = int(y // delta)
		return self.is_empt(i, j)

	def index1(self, i, j):
		return i * self.j_range + j

	def index2(self, k):
		a = [0, 0]
		a[0] = k // self.j_range
		a[1] = k % self.j_range
		return a

	def loop_index_ard(self, i = 0, j = 0, N = 0):
		index = []
		if N == 0: N = self.i_range + self.j_range - 2
		for l in range(1, N + 1):
			for k in range(l):
				index.append([i + k, j + l - k])
				index.append([i + l - k, j - k])
				index.append([i - k, j - l + k])
				index.append([i - l + k, j + k])
		return index

	def nbr_rough(self, i, j):
		v_nb = []
		if not self.is_empt(i, j) : return v_nb
		if self.is_empt(i, j + 1) : v_nb.append(self.index1(i, j + 1))
		if self.is_empt(i + 1, j + 1) : v_nb.append(self.index1(i + 1, j + 1))
		if self.is_empt(i + 1, j) : v_nb.append(self.index1(i + 1, j))
		if self.is_empt(i + 1, j - 1) : v_nb.append(self.index1(i + 1, j - 1))
		if self.is_empt(i, j - 1) : v_nb.append(self.index1(i, j - 1))
		if self.is_empt(i - 1, j - 1) : v_nb.append(self.index1(i - 1, j - 1))
		if self.is_empt(i - 1, j) : v_nb.append(self.index1(i - 1, j))
		if self.is_empt(i - 1, j + 1) : v_nb.append(self.index1(i - 1, j + 1))
		return v_nb

	def nbr_smooth(self, i, j, loop_index = None):
		v_nb = []
		if not self.is_empt(i, j) : return v_nb
		N = int(r_nbr // delta)
		if loop_index is None:
			loop_index = self.loop_index_ard(0, 0, N)
		v_st = [[1]*(2*N + 3) for p in range(2*N + 3)]
		for [k, l] in loop_index:
			if self.is_empt(i + k, j + l):
				v_st[k][l] = v_st[k + 1][l]*v_st[k][l + 1]*v_st[k - 1][l]*v_st[k][l - 1]
			else:
				v_st[k][l] = 0
			if v_st[k][l] == 1: v_nb.append(self.index1(i + k, j + l))
		return v_nb

	def nbr(self, i, j):
		"Return the neighbour of vertex whose status is empty. if the status of v[i][j] is not empty, then the neighbour is empty"
		try:
			return self.edge[self.index1(i, j)]
		except:
			if graph_mode == ROUGH:
				v_nb = self.nbr_rough(i, j)
			elif graph_mode == SMOOTH:
				v_nb = self.nbr_smooth(i, j)
			return v_nb

	def nbr2(self, k):
		try:
			return self.edge[k]
		except:
			[i, j] = self.index2(k)
			if graph_mode == ROUGH:
				v_nb = self.nbr_rough(i, j)
			elif graph_mode == SMOOTH:
				v_nb = self.nbr_smooth(i, j)
			return v_nb

	def dist(self, k, l):
		a = self.index2(k)
		b = self.index2(l)
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def dist2(self, a, b):
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def add_rect(self, x, y, w, h):
		"Parameters' type is float (cm)"
		if self.finished:
			return
		w += 2 * r_robot
		h += 2 * r_robot
		l = int(max((x - w / 2) / delta, 0))
		if delta * l < x - w: l += 1
		r = int(min((x + w / 2) / delta, self.j_range - 1))
		u = int(max((y - h / 2) / delta, 0))
		if delta * u < y - h: u += 1
		d = int(min((y + h / 2) / delta, self.i_range - 1))
		for i in range(u, d + 1):       # i: rows
			for j in range(l, r + 1):   # j: columns
				self.v[i][j] = v_type.full
		return

	def add_circle(self, x, y, rad):
		"Parameters' type are float (cm)"
		if self.finished:
			return
		rad += r_robot
		l = int(max((x - rad) / delta, 0))
		r = int(min((x + rad) / delta, self.j_range - 1))
		u = int(max((y - rad) / delta, 0))
		d = int(min((y + rad) / delta, self.i_range - 1))
		for i in range(u, d + 1):       # i: rows
			for j in range(l, r + 1):   # j: columns
				if (j * delta - x)**2 + (i * delta - y)**2 <= rad**2 : # j for x-axes, i for y-axes
					self.v[i][j] = v_type.full
		return

	def finish(self):
		self.finished = True
		self.prt_map()
		print "Computing the edges..."
		self.edge = []
		if graph_mode == SMOOTH:
			N = int(r_nbr // delta)
			loop_index = self.loop_index_ard(0, 0, N)
			for k in range(self.N):
				[i, j] = self.index2(k)
				self.edge.append(self.nbr_smooth(i, j, loop_index))
		elif graph_mode == ROUGH:
			for k in range(self.N):
				[i, j] = self.index2(k)
				self.edge.append(self.nbr_rough(i, j))
		print "The graph has constructed successfully!"
		return

	def correct_point_index(self, i, j, check_func = None):
		if check_func is None:
			check_func = self.is_empt
		if i < 0:
			i = 0
		elif i >= self.i_range:
			i = self.i_range - 1
		if j < 0:
			j = 0
		elif j >= self.j_range:
			j = self.j_range - 1
		if check_func(i, j):
			return [i, j]
		for [k, l] in self.loop_index_ard(i, j):
			if check_func(k, l):
				return [k, l]
		return []

	def correct_point_coordinate(self, x, y, check_func = None):
		j = int(x // delta)
		i = int(y // delta)
		[i, j] = self.correct_point_index(i, j, check_func)
		return [j * delta, i * delta]

	def find_path(self, px, py, qx, qy):
		pj = int(px // delta)
		pi = int(py // delta)
		qj = int(qx // delta)
		qi = int(qy // delta)

		# Correct the point
		if not self.is_empt(pi, pj):
			[pi, pj] = self.correct_point_index(pi, pj) # correct the origin
			if self.log is None:
				print "Navi: The origin is illegal!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(px, py, pj * delta, pi * delta)
			else:
				self.log.logwarn("Navi: The origin is illegal!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(px, py, pj * delta, pi * delta))
		if not self.is_empt(qi, qj):
			[qi, qj] = self.correct_point_index(qi, qj) # correct the destination
			if self.log is None:
				print "Navi: The destination is illegal!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(qx, qy, qj * delta, qi * delta)
			else:
				self.log.logwarn("Navi: The destination is illegal!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(qx, qy, qj * delta, qi * delta))
			qx = qj*delta
			qy = qi*delta

		[target, path] = self.find_path_graph(pi, pj, qi, qj)
		if len(path) == 0:
			return path
		if target != self.index1(qi, qj):
			if self.log is None:
				print "Navi: The destination is not accessible!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(qx, qy, path[-1][0], path[-1][1])
			else:
				self.log.logwarn("Navi: The destination is not accessible!\nAutomatically correct it from (%s, %s) to (%s, %s)"%(qx, qy, path[-1][0], path[-1][1]))
		else:
			path[-1] = [qx, qy] # destination
		return path

	def dijkstra(self, pk, qk):
		"Use dijsktra algorithm to determine the shortest path for single origin. Input the index of vertex, and return the path with unit cm"
		v_st = [True]*self.N # store whether the distance has been measured
		for i in range(self.i_range):
			for j in range(self.j_range):
				v_st[self.index1(i, j)] = (self.v[i][j] != v_type.empt)
		dis = [DIST_MAX]*self.N
		d = DIST_MAX
		dis[pk] = 0
		active = my_pq()
		active.put(pk) # store the vertexes which have the estimation of distance but not sure(active vertexes)
		target = -1

		# Dijkstra algorithm
		while not active.empty():
			cur_v = active.get()
			v_st[cur_v] = True
			if d > self.dist(cur_v, qk):
				d = self.dist(cur_v, qk) # record the distance to dst
				target = cur_v
			if d < eps: 	# if the distance is 0
				break  		# then break the loop as we get the dst
			for i in self.nbr2(cur_v):
				if dis[i] == DIST_MAX:
					dis[i] = dis[cur_v] + self.dist(i, cur_v)
					active.put(i, dis[i])
				else:
					dis[i] = min(dis[i], dis[cur_v] + self.dist(i, cur_v))
					if not v_st[i]:
						active.update(i, dis[i])
		return [target, dis]

	def find_shortest_path(self, dis, target_i, target_j):
		"Find the shortest path to target by distance list from origin. Path do not contain the origin."
		path = []
		p = []
		v_path = [target_i, target_j] # current vertex
		lenth = dis[self.index1(v_path[0], v_path[1])]
		path.append([v_path[1]*delta, v_path[0]*delta])
		p.append([v_path[0], v_path[1]])
		cur_len = dis[self.index1(target_i, target_j)]
		while cur_len > 1e-12:
			if len(p) >= 2:
				v_tmp = [2*p[-1][0] - p[-2][0], 2*p[-1][1] - p[-2][1]]
				if self.is_empt(v_tmp[0], v_tmp[1]) and abs(cur_len - dis[self.index1(v_tmp[0], v_tmp[1])] - self.dist2(v_tmp, v_path)) < eps:
					v_path = v_tmp
					p.append([v_path[0], v_path[1]])
					path[-1] = [v_path[1]*delta, v_path[0]*delta]
					cur_len = dis[self.index1(v_path[0], v_path[1])]
					continue
			for k in self.nbr(v_path[0], v_path[1]):
				if abs(cur_len - dis[k] - self.dist2(self.index2(k), v_path)) < eps:
					v_path = self.index2(k)
					p.append([v_path[0], v_path[1]])
					path.append([v_path[1]*delta, v_path[0]*delta])
					cur_len = dis[self.index1(v_path[0], v_path[1])]
					break
		if len(path) > 1:
			path.pop()
		path.reverse()
		return [path, p]

	def find_path_graph(self, pi, pj, qi, qj):
		if (not self.is_empt(pi, pj)) or (not self.is_empt(qi, qj)):
			return []
		pk = self.index1(pi, pj)
		qk = self.index1(qi, qj)
		[target, dis] = self.dijkstra(pk, qk)
		[ti, tj] = self.index2(target)
		[path, p] = self.find_shortest_path(dis, ti, tj)
		self.prt_path(p)
		return [target, path]

	def prt_map(self):
		if self.j_range > 100:
			return
		str = ["○", "●"]
		print "\n"
		for i in range(self.i_range):
			for j in range(self.j_range):
				print str[self.v[-i - 1][j].value],
			print "\n",
		print "\n"

	def prt_path(self, path):
		if self.j_range > 100:
			return
		str = ["○", "●", "▼"]
		v = [[0]*self.j_range for i in range(self.i_range)]
		for i in range(self.i_range):
			for j in range(self.j_range):
				v[i][j] = self.v[i][j].value
		for p in path:
			v[p[0]][p[1]] = 2;
		print "\n"
		for i in range(self.i_range):
			for j in range(self.j_range):
				print str[v[-i - 1][j]],
			print "\n",
		print "\n"

def test_grid_graph():
	failed_str = ["Cannot correctly determine whether a point is legal!", "Cannot correctly find the path!"]
	m = grid_graph(110, 500)
	m.add_rect(13, 64, 30, 30)
	m.add_rect(63, 424, 45, 40)
	m.add_rect(73, 200, 20, 140)
	m.add_rect(-100, 200, 96, 62)
	m.add_circle(50, 50, 25)
	m.add_circle(42, 313, 50)
	m.add_circle(200, 200, 63)
	m.finish()
	m.add_circle(0, 0, 100)
	m.correct_point_coordinate(20, 50)
	path = m.find_path(0, 0, 83, 413)
	assert m.is_empt_coordinate(0, 0), failed_str[0]
	assert m.is_empt_coordinate(99, 499), failed_str[0]
	assert not m.is_empt_coordinate(-1, 499), failed_str[0]
	assert not m.is_empt_coordinate(0, 500), failed_str[0]
	assert not m.is_empt_coordinate(93, 50), failed_str[0]
	assert not m.is_empt_coordinate(61, 134), failed_str[0]
	assert len(path) > 0, failed_str[1]
	path = m.find_path(0, 0, 0, 0)
	assert len(path) > 0, failed_str[1]

if __name__ == '__main__':
	try:
		test_grid_graph()
		print "Test pass!"
	except AssertionError as e:
		print e
	
