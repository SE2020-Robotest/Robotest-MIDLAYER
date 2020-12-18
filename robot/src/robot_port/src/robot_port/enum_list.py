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
eps_d = 2.0 # The tollerance
eps_a = 0.5 # The tollerance
max_l = 0.3			# The linear velocity is bounded by max_l.
max_a = 1.0			# The angular velocity is bounded by max_a.
max_delta_l = 0.04 	# The difference of linear velocity is bounded by max_delta_l.
max_delta_a = 0.1 	# The difference of angular velocity is bounded by max_delta_a.

# Grid_graph
DIST_MAX = 9999.0
eps = 1e-12
ROUGH = 0
SMOOTH = 1

r_robot = 18.0 # Radius of robot
r_nbr = 50.0
delta = 5.0 # grid size
graph_mode = SMOOTH

# Navi: move_to_user
d_user_min = 100		#the minimal distance to the user while recogniting
d_user_max = 200	#the maximal distance to the user while recogniting

# Voice_cmd:
v_cmd = {
	"stop": ["stop", "停止。", "终止。", "停止实验。", "终止实验。"]
	"spin": ["spin", "旋转。", "开始旋转。", "原地旋转。"]
	"stop spinning": ["stop spinning", "停止旋转。", "终止旋转。", "结束旋转。"]
	"look me": ["look me", "看我。", "转过来。", "看过来。", "嗨。"]
	"come here": ["come here", "过来。", "走过来。", "快过来。", "来。", "到这来。"]
	"move to origin": ["move to origin", "回到原点。", "回原点。", "回去。", "回去原点。", "去原点。"]
	
}
