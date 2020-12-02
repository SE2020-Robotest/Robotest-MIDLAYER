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

r_robot = 5.0 # Radius of robot
delta = 10 # grid size
DIST_MAX = 9999.0
eps = 1e-4

class v_type(Enum):
    empt = 0
    full = 1

class grid_graph:

    def __init__(self, w = 100, h = 100):
        self.w = int(w / delta)
        self.h = int(h / delta)
        self.N = self.w * self.h
        self.v = [[v_type.empt]*self.w for i in range(self.h)]
        self.finished = False;
        return

    def is_inside(self, i, j):
        "Determine whether i,j is in the graph"
        return 0 <= i and i < self.w and 0 <= j and j < self.h

    def is_empt(self, i, j):
        "Determine whether i,j is in the graph, and v[i][j] is empty"
        return self.is_inside(i, j) and self.v[i][j] == v_type.empt

    def index1(self, i, j):
        return i * self.w + j

    def index2(self, k):
        a = [0, 0]
        a[0] = k // self.w
        a[1] = k % self.w
        return a

    def nbr(self, i, j):
        "Return the neighbour of vertex whose status is empty. if the status of v[i][j] is not empty, then the neighbour is empty"
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

    def nbr2(self, k):
        a = self.index2(k)
        return self.nbr(a[0], a[1])

    def dist(self, k, l):
	a = self.index2(k)
	b = self.index2(l)
	return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def dist2(self, a, b):
	return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def add_rect(self, x, y, w, h):
        "Parameters' type are float (cm)"
        if self.finished:
            return
        w += r_robot
        h += r_robot
        l = int(max((x - w / 2) / delta, 0))
        if delta * l < x - w: l += 1
        r = int(min((x + w / 2) / delta, self.w - 1))
        u = int(max((y - h / 2) / delta, 0))
        if delta * u < y - h: u += 1
        d = int(min((y + h / 2) / delta, self.h - 1))
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
        r = int(min((x + rad) / delta, self.w - 1))
        u = int(max((y - rad) / delta, 0))
        d = int(min((y + rad) / delta, self.h - 1))
        for i in range(u, d + 1):       # i: rows
            for j in range(l, r + 1):   # j: columns
                if (j * delta - x)**2 + (i * delta - y)**2 <= rad**2 : # j for x-axes, i for y-axes
                    self.v[i][j] = v_type.full
        return

    def finish(self):
        self.finished = True
	self.prt_map()
        return

    def correct_point_index(self, i, j):
	if i < 0:
	    i = 0
	elif i >= self.h:
	    i = self.h - 1
	if j < 0:
	    j = 0
	elif j >= self.w:
	    j = self.w - 1
	if self.is_empt(i, j):
	    return [i, j]
	for l in range(1, self.w + self.h):
	    for k in range(l):
		if self.is_empt(i + k, j + l - k):
	    	    return [i + k, j + l - k]
		elif self.is_empt(i + l - k, j - k):
	    	    return [i + l - k, j - k]
		elif self.is_empt(i - k, j - l + k):
	    	    return [i - k, j - l + k]
		elif self.is_empt(i - l + k, j + k):
	    	    return [i - l + k, j + k]

    def correct_point_coordinate(self, x, y):
	j = int(x // delta)
	i = int(y // delta)
	[i, j] = self.correct_point_index(i, j)
	return [j * delta, i * delta]

    def is_empt_coordinate(self, x, y):
	j = int(x // delta)
	i = int(y // delta)
	return self.is_empt(i, j)

    def find_path(self, px, py, qx, qy):
	pj = int(px // delta)
	pi = int(py // delta)
	qj = int(qx // delta)
	qi = int(qy // delta)

	# Correct the point
	if not self.is_empt(pi, pj):
	    [pi, pj] = self.correct_point_index(pi, pj) # correct the origin
	    if __name__ == '__main__':
	        print "Navi: The origin is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]"%(px, py, pj * delta, pi * delta)
	    else:
		logwarn("Navi: The origin is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]"%(px, py, pj * delta, pi * delta))
	if not self.is_empt(qi, qj):
	    [qi, qj] = self.correct_point_index(qi, qj) # correct the destination
	    if __name__ == '__main__':
	        print "Navi: The destination is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]"%(qx, qy, qj * delta, qi * delta)
	    else:
		logwarn("Navi: The destination is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]"%(qx, qy, qj * delta, qi * delta))
	    qx = qj*delta
	    qy = qi*delta

	path = self.find_path_graph(pi, pj, qi, qj)
	if len(path) == 0:
	    return path
	if (path[-1][0] - qx)**2 + (path[-1][1] - qy)**2 > eps**2:
	    path.append([qx, qy]) # destination
	return path

    def find_path_graph(self, pi, pj, qi, qj):
        "Use dijsktra algorithm to determine the shortest path for single origin. Input the index of vertex, and return the path with unit cm"
        path = []
        if (not self.is_empt(pi, pj)) or (not self.is_empt(qi, qj)):
            return path
        v_st = [True]*self.N # store whether the distance has been measured
        for i in range(self.h):
            for j in range(self.w):
                if self.v[i][j] == v_type.empt:
                    v_st[self.index1(i, j)] = False
                else:
                    v_st[self.index1(i, j)] = True
        dis = [DIST_MAX]*self.N
        dis[self.index1(pi, pj)] = 0
        active = [self.index1(pi, pj)] # store the vertexes which have the estimation of distance but not sure(active vertexes)

        # Dijkstra algorithm
        for k in range(self.N):
            mark = -1
            min_dis = DIST_MAX
            it = 0      # the index in active vertexes
            p_min = 0   # the index where achieve the minimum distance
            for i in active:
                if (not v_st[i]) and (dis[i] < min_dis):
                    min_dis = dis[i]
                    mark = i
                    p_min = it
                it += 1
            if mark == -1:  # if mark == -1, then active is empty, and the destination is not accessable
                return path # therefore return empty path
            v_st[mark] = True
            del active[p_min]
            if mark == self.index1(qi, qj):
                break
            for i in self.nbr2(mark):
                if dis[i] == DIST_MAX:
                    active.append(i)
                dis[i] = min(dis[i], dis[mark] + self.dist(i, mark))

        # find the shortest path
        v_path = [qi, qj] # current vertex
        lenth = dis[self.index1(v_path[0], v_path[1])]
	p = []
        path.append([v_path[1]*delta, v_path[0]*delta])
	p.append([v_path[0], v_path[1]])
	cur_len = dis[self.index1(qi, qj)]
        while cur_len > 1e-12:
	    if len(p) >= 2:
		v_tmp = [2*p[-1][0] - p[-2][0], 2*p[-1][1] - p[-2][1]]
		if self.is_empt(v_tmp[0], v_tmp[1]) and abs(cur_len - dis[self.index1(v_tmp[0], v_tmp[1])] - self.dist2(v_tmp, v_path)) < 1e-12:
		    v_path = v_tmp
		    p.append([v_path[0], v_path[1]])
		    path[-1] = [v_path[1]*delta, v_path[0]*delta]
		    cur_len = dis[self.index1(v_path[0], v_path[1])]
		    continue
            for k in self.nbr(v_path[0], v_path[1]):
                if abs(cur_len - dis[k] - self.dist2(self.index2(k), v_path)) < 1e-12:
                    v_path = self.index2(k)
		    p.append([v_path[0], v_path[1]])
        	    path.append([v_path[1]*delta, v_path[0]*delta])
		    cur_len = dis[self.index1(v_path[0], v_path[1])]
                    break
        path.reverse()
	self.prt_path(p)
        return path

    def prt_map(self):
	str = ["○", "●"]
        print "\n"
    	for i in range(self.h):
            for j in range(self.w):
                print str[self.v[-i - 1][j].value],
            print "\n",
        print "\n"

    def prt_path(self, path):
	str = ["○", "●", "▼"]
	v = [[0]*self.w for i in range(self.h)]
        for i in range(self.h):
            for j in range(self.w):
                v[i][j] = self.v[i][j].value
	for p in path:
            v[p[0]][p[1]] = 2;
        print "\n"
    	for i in range(self.h):
            for j in range(self.w):
                print str[v[-i - 1][j]],
            print "\n",
        print "\n"


if __name__ == '__main__':
    # Test
    m = grid_graph(500, 500)
    m.add_circle(242, 313, 98)
    m.add_rect(123, 234, 100, 120)
    m.add_rect(323, 324, 90, 40)
    m.add_rect(423, 224, 200, 160)
    m.add_circle(50, 50, 98)
    m.finish()
    path = m.find_path(500, 100, 283, 413)
    for p in path:
	print p[0], p[1]

