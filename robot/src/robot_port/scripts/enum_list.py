#!/usr/bin/env python

# Message
CUBE = 0
CYLINDER = 1

START = 0
STOP = 1
CONNECT = 2

FRONT = 0
BACK = 1
LEFT = 2
RIGHT = 3
CLOCKWISE = 4
ANTICLOCKWISE = 5

OK = 0
ERROR = 1
FINISHED = 2

# Moving settings
eps_d = 2 # The tollerance
eps_a = 0.5 # The tollerance
d_slow = 10 # When the distance is lower than d_slow, the robot will reduce the speed.
d_turn = 3 # When the distance is lower than d_turn, the robot will turn around.
max_delta_l = 0.05
max_l = 0.2
max_a = 1

# Grid_graph
DIST_MAX = 9999.0
eps = 1e-12
ROUGH = 0
SMOOTH = 1

r_robot = 18.0 # Radius of robot
r_nbr = 50
delta = 5 # grid size
graph_mode = SMOOTH
