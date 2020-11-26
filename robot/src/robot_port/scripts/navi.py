#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
地图采用的单位为cm，原点为左上角
'''

from enum import Enum

r_robot = 5
delta = 10 # 栅格精度
INT_MAX = 0xfffffff

class v_type(Enum):
    empt = 0
    full = 1

class map:
    def __init__(self, w, h):
        self.w = int(w / delta)
        self.h = int(h / delta)
        self.N = self.w * self.h
        self.v = [[v_type.empt]*self.w for i in range(self.h)]
        self.finished = False;
        return super().__init__()
    def is_inside(self, i, j):
        "判断i,j是否在地图内"
        return 0 <= i and i < self.w and 0 <= j and j < self.h
    def is_empt(self, i, j):
        "判断i,j是否在地图内，以及v[i][j]是否状态为empt"
        return self.is_inside(i, j) and self.v[i][j] == v_type.empt
    def index1(self, i, j):
        return i * self.w + j
    def index2(self, k):
        a = [0, 0]
        a[0] = k // self.w
        a[1] = k % self.w
        return a
    def nbr(self, i, j):
        "返回v[i][j]的状态为empt的领域。如果v[i][j]本身状态不是empt则领域为空"
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
        "传入参数类型为float，单位为cm"
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
        for i in range(u, d + 1):       # i代表行数
            for j in range(l, r + 1):   # j代表列数
                self.v[i][j] = v_type.full
        return
    def add_circle(self, x, y, rad):
        "传入参数类型为float，单位为cm"
        if self.finished:
            return
        rad += r_robot
        l = int(max((x - rad) / delta, 0))
        r = int(min((x + rad) / delta, self.w))
        u = int(max((y - rad) / delta, 0))
        d = int(min((y + rad) / delta, self.h))
        for i in range(u, d + 1):       # i代表行数
            for j in range(l, r + 1):   # j代表列数
                if (j * delta - x)**2 + (i * delta - y)**2 <= rad**2 : # j对应横坐标，i对应纵坐标
                    self.v[i][j] = v_type.full
        return
    def finish(self):
        self.finished = True
        return
    def find_path(self, pi, pj, qi, qj):
        "采用dijkstra算法计算单源最短路"
        path = []
        if (not self.is_empt(pi, pj)) or (not self.is_empt(qi, qj)):
            return path
        v_st = [True]*self.N # 记录是否已确定距离
        for i in range(self.h):
            for j in range(self.w):
                if self.v[i][j] == v_type.empt:
                    v_st[self.index1(i, j)] = False
                else:
                    v_st[self.index1(i, j)] = True
        dis = [INT_MAX]*self.N
        dis[self.index1(pi, pj)] = 0
        active = [self.index1(pi, pj)] # 记录当前已经有距离估计但未完全确定的节点(活跃节点)

        # 以下为dijkstra算法
        for k in range(self.N):
            mark = -1
            min_dis = INT_MAX
            it = 0      # 记录遍历active中元素时的位置
            p_min = 0   # 记录active中达到min的相应位置
            for i in active:
                if (not v_st[i]) and (dis[i] < min_dis):
                    min_dis = dis[i]
                    mark = i
                    p_min = it
                it += 1
            if mark == -1:  # 如果mark == -1，说明active空，目标点不可达
                return path # 因此返回空路径
            v_st[mark] = True
            del active[p_min]
            if mark == self.index1(qi, qj):
                break
            for i in self.nbr2(mark):
                if dis[i] == INT_MAX:
                    active.append(i)
                dis[i] = min(dis[i], dis[mark] + 1)

        # 以下计算最短路径
        v_path = self.index1(qi, qj)
        len = dis[v_path]
        path.append(self.index2(v_path))
        for cur_len in range(len - 1, 0, -1):
            for k in self.nbr2(v_path):
                if dis[k] == cur_len:
                    v_path = k
                    path.append(self.index2(v_path))
                    break
        path.append([pi, pj])
        path.reverse()
        return path

# 实例测试
m = map(500, 500)
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
        print(str[tmp[i][j]], end = '')
    print("\n", end = '')
print("\n")

#while True:
#    pi = int(input("输入起点行：\n"))
#    pj = int(input("输入起点列：\n"))
#    qi = int(input("输入终点行：\n"))
#    qj = int(input("输入终点列：\n"))
#    print(m.find_path(pi, pj, qi, qj))
#    print("\n")
