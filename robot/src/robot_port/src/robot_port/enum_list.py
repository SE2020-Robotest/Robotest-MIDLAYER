#!/usr/bin/env python
# -*- coding: utf-8 -*-


######## Message ########

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

#########################


################################ Moving settings ###################################

rough_eps_d = 2.0 	# The loose tollerance
rough_eps_a = 0.2 	# The loose tollerance
accu_eps_d = 2.0 	# The accurate tollerance
accu_eps_a = 0.03 	# The accurate tollerance
max_l = 0.3			# The linear velocity is bounded by max_l.
max_a = 1.0			# The angular velocity is bounded by max_a.
max_delta_l = 0.04 	# The difference of linear velocity is bounded by max_delta_l.
max_delta_a = 0.1 	# The difference of angular velocity is bounded by max_delta_a.

####################################################################################


################################## Grid_graph ######################################

DIST_MAX = 9999.0
eps = 1e-12
ROUGH = 0
SMOOTH = 1


r_robot = 18.0 		# Radius of robot
r_nbr = 50.0
delta = 5.0 		# Grid size
graph_mode = SMOOTH

####################################################################################


############################## Posi_Publisher: rate ##################################

pub_rate = 5 		# HZ

####################################################################################


############################## Navi: move_to_user ##################################

d_user_min = 120	# The minimal distance to the user while recogniting
d_user_max = 170	# The maximal distance to the user while recogniting

####################################################################################


############################ TF transform: frame_id ################################

VMAP_FRAME = "vmap"
ROBOT_FRAME = "base_link"

####################################################################################

################################################ voice_cmd ####################################################

v_cmd = {
	"stop": {"stop", "停止", "终止", "停止实验", "终止实验"},
	"spin": {"spin", "旋转", "开始旋转", "原地旋转"},
	"stop spinning": {"stop spinning", "停止旋转", "终止旋转", "结束旋转"},
	"look me": {"look me", "看我", "转过来", "看过来", "嗨"},
	"come here": {"come here", "过来", "走过来", "快过来", "到这来"},
	"move to origin": {"move to origin", "回到原点", "回原点", "回去", "回去原点", "去原点"},
	"move to the door": {"move to the door", "到门边", "去门边", "去门那"},
	"move to there": {"move to there", "去那里", "去哪里", "到那里", "到哪里", "去那", "到那去", "到那里去"}
}

cmd_list = {"stop", "停止", "终止", "停止实验", "终止实验", "spin", "旋转", "开始旋转", "原地旋转", "stop spinning",\
			"停止旋转", "终止旋转", "结束旋转", "look me", "看我", "转过来", "看过来", "嗨", "come here", "过来",\
			"走过来", "快过来", "到这来", "move to origin", "回到原点", "回原点", "回去", "回去原点", "去原点",\
			"move to the door", "到门边", "去门边", "去门那", "move to there", "去那里", "去哪里", "到那里",\
			"到哪里", "去那", "到那去", "到那里去"}

###############################################################################################################


###################### voice_pub ##########################

illegal_char = ("。", "！", "？", "儿", "，", "嗯")

###########################################################

################# Gesture and convert #####################

CAMERA_THETA = 35 	# degree
CAMERA_X = 0.1		# meter
CAMERA_Y = -0.01	# meter
CAMERA_Z = 0.23		# meter
BAD_POINT = -0xffff

###########################################################
