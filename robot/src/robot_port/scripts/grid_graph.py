#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
The origin point is on the bottom-left, and unit is cm

i          y
|          |
|       &  |
|_____ j   |_____ x

This is the corresbonding relation between grid_graph and virtual_map
'''

from enum import Enum

r_robot = 5.0 # Radius of robot
delta = 10 # grid size
INT_MAX = 0xfffffff

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
        if j + 1 < self.w and self.v[i][j + 1] == v_type.empt : v_nb.append(self.index1(i, j + 1))
        if i - 1 >= 0 and self.v[i - 1][j] == v_type.empt : v_nb.append(self.index1(i - 1, j))
        if j - 1 >= 0 and self.v[i][j - 1] == v_type.empt : v_nb.append(self.index1(i, j - 1))
        if i + 1 < self.h and self.v[i + 1][j] == v_type.empt : v_nb.append(self.index1(i + 1, j))
        return v_nb

    def nbr2(self, k):
        a = self.index2(k)
        return self.nbr(a[0], a[1])

    def add_rect(self, x, y, w, h):
        "Parameters' type are float (cm)"
        if self.finished:
            return
        w += r_robot
        h += r_robot
        l = int(max((x - w) / delta, 0))
        if delta * l < x - w: l += 1
        r = int(min((x + w) / delta, self.w - 1))
        u = int(max((y - h) / delta, 0))
        if delta * u < y - h: u += 1
        d = int(min((y + h) / delta, self.h - 1))
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
        r = int(min((x + rad) / delta, self.w))
        u = int(max((y - rad) / delta, 0))
        d = int(min((y + rad) / delta, self.h))
        for i in range(u, d + 1):       # i: rows
            for j in range(l, r + 1):   # j: columns
                if (j * delta - x)**2 + (i * delta - y)**2 <= rad**2 : # j for x-axes, i for y-axes
                    self.v[i][j] = v_type.full
        return

    def finish(self):
        self.finished = True
	self.prt_map()
        return

    def correct_point(self, i, j):
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

    def find_path(self, pi, pj, qi, qj):
        "Use dijsktra algorithm to determine the shortest path for single origin"
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
        dis = [INT_MAX]*self.N
        dis[self.index1(pi, pj)] = 0
        active = [self.index1(pi, pj)] # store the vertexes which have the estimation of distance but not sure(active vertexes)

        # Dijkstra algorithm
        for k in range(self.N):
            mark = -1
            min_dis = INT_MAX
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
                if dis[i] == INT_MAX:
                    active.append(i)
                dis[i] = min(dis[i], dis[mark] + 1)

        # find the shortest path
        v_path = self.index1(qi, qj)
        len = dis[v_path]
        path.append(self.index2(v_path))
        for cur_len in range(len - 1, 0, -1):
            for k in self.nbr2(v_path):
                if dis[k] == cur_len:
                    v_path = k
                    path.append(self.index2(v_path))
                    break
        path.reverse()
	self.prt_path(path)
        return path

    def prt_map(self):
	str = ["○", "●"]
        print "\n"
    	for i in range(m.h):
            for j in range(m.w):
                print str[self.v[i][j].value],
            print "\n",
        print "\n"

    def prt_path(self, path):
	str = ["○", "●", "▼"]
	v = [[0]*m.w for i in range(m.h)]
        for i in range(m.h):
            for j in range(m.w):
                v[i][j] = m.v[i][j].value
	for p in path:
            v[p[0]][p[1]] = 2;
        print "\n"
    	for i in range(m.h):
            for j in range(m.w):
                print str[v[i][j]],
            print "\n",
        print "\n"


if __name__ == '__main__':
    # Test
    m = grid_graph(500, 500)
    m.add_circle(242, 313, 98)
    m.add_rect(123, 234, 50, 67)
    m.add_rect(323, 324, 45, 20)
    m.add_rect(423, 224, 100, 80)
    
    str = ["○", "●", "▼"]
    v = [[0]*m.w for i in range(m.h)]
    for i in range(m.h):
        for j in range(m.w):
            v[i][j] = m.v[i][j].value
            #print(str[v[i][j]], end = '')
        #print("\n", end = '')
    print("\n")
    tmp = v
    for p in m.find_path(10, 40, 49, 49):
        tmp[p[0]][p[1]] = 2;
    for i in range(m.h):
        for j in range(m.w):
            print str[tmp[i][j]],
        print "\n",
    print("\n")

