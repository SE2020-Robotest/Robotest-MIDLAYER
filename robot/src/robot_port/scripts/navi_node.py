#!/usr/bin/env python


import rospy
from status import rb, exp
import grid_graph as graph
from trans import trans
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import pi, atan2, abs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import path
from robot_port.msg import point_2d

'''
The origin point is on the bottom-left, and unit is cm

i           y
|           |
|        &  |
|_____ j    |_____ x

This is the corresbonding relation between grid_graph and virtual_map
'''

eps = 0.0001 # The tollerance
class navi_method(Enum):
    cmd_vel = 0
    move_base_grid_graph = 1
    move_base_map_server = 2

cur_navi_method = navi_method.cmd_vel

class navi_nodes:
    def __init__(self):
	self.map_loaded = False
	self.g = None
	self.rb_s = rb()
	self.exp_s = exp()

	self.move_cmd = Twist()
	self.move_cmd_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	self.path_pub = rospy.Publisher('send_rb_path', path, queue_size = 10)

	rospy.init_node('navi_node', anonymous = False)
        rospy.loginfo("Navi: Navigation Node Initialized!")

	rospy.Subscriber('virtual_map', vmap, self.load_map)
	rospy.Subscriber('path_message', path, self.recieve_path)
	rospy.Subscriber('dst_message', point_2d, self.recieve_dst)

	r = rospy.get_param("robot_radius")
	d = rospy.get_param("grid_size")
	graph.r_robot = float(r)
	graph.delta = d
	return

    def start(self):
	"Start running"
	rospy.spin()
	return

    def load_map(self, msg):
	"Load the map message from Control port"
	'''
	map_object:
	    bool type 	# False means CUBE, True means CYLINDER
	    float64 x
	    float64 y
	    float64 w
	    float64 h
	vmap:
	    float32 w
	    float32 h
	    map_object[] obj
	'''
	rb_status = self.rb_s.get_status()
	rospy.loginfo("Navi: Loading the map")
	if rb_status != rb.init:
	    rospy.logerr("Navi: Cannot init the map! The Exp hasn't been initialized.")
	    return
	self.g = graph.grid_graph(msg.w, msg.h)
	for obj in msg.obj:
	    if obj.type == 1:
		self.g.add_circle(obj.x, obj.y, obj.w)
	    else:
		self.g.add_rect(obj.x, obj.y, obj.w, obj.h)
	self.g.finish()
	rospy.loginfo("Navi: Load the map successfully!")
	self.rb_s.Run()							# (Important!) Change the status
	return

    def recieve_path(self, msg):
	"Recieve the path, correct them and move along this path. Finally, send the corrected path to Control port"
	rb_status = self.rb_s.get_status()
	exp_status = self.exp_s.get_status()
	if rb_status != rb.run or exp_status != rb.wait:
	    return
	self.exp_s.Move() 						# (Important!) Change the status

	
	return

    def recieve_dst(self, msg):
	"Recieve the destination, correct it and find a path to move along. Finally, send the path to Control port"
	'''
	path:
	    point_2d[] p
	point_2d:
	    float64 x
	    float64 y
	'''
	
	# Check the status
	rb_status = self.rb_s.get_status()
	exp_status = self.exp_s.get_status()
	if rb_status != rb.run or exp_status != rb.wait:
	    return
	self.exp_s.Move() 						# (Important!) Change the status

	# Correct the point
	qx = msg.x # destination
	qy = msg.y # destination
	qj = int(qx)
	qi = int(qy)
	if not self.g.is_empt(qi, qj):
	    [ti, tj] = self.g.correct_point(qi, qj) # correct the destination
	    rospy.logerr("Navi: The destination is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]", qi, qj, ti, tj)
	    qy = qi = ti
	    qx = qj = tj
	# Get the Robot's position
	try:
	    posi_msg = rospy.wait_for_message('/odom', Odometry, timeout = 5)
	except rospy.ROSInterruptException:
	    return
	except:
	    rospy.logerr("Navi: Cannot get the Robot's position!")
	    self.exp_s.Wait() 						# (Important!) Change the status
	    return
	px = posi_msg.pose.pose.position.x # origin
	py = posi_msg.pose.pose.position.y # origin
	pj = int(px)
	pi = int(py)
	[pi, pj] = self.g.correct_point(pi, pj) # correct the origin

	# Find path
	path = self.g.find_path(pi, pj, qi, qj)
	if len(path) == 0:
	    rospy.logerr("Navi: The destination is not accessable!")
	    self.exp_s.Wait() 						# (Important!) Change the status
	    return
	if (path[-1][0] - qx)**2 + (path[-1][1] - qy)**2 > eps**2:
	    path.append([qx, qy]) # destination

	# Send path message
	path_msg = []
	path_mag.p.append(point_2d(px, py)) # origin
	for p in path:
	    path_msg.append(point_2d(p[0], p[1]))
	self.path_pub.publish(path_msg)

	# Move to the destination
	self.move(path)

	rospy.loginfo("Navi: Arrive at destination!")
	self.exp_s.Wait() 						# (Important!) Change the status
	return

    def move(self, path):
	if cur_navi_method == navi_method.cmd_vel:
	    self.move_by_cmd_vel(path)
	elif cur_navi_method == navi_method.move_base_grid_graph:
	    self.move_by_move_base_grid_graph(path)
	elif cur_navi_method == navi_method.move_base_map_server:
	    self.move_by_move_base_map_server(path)

    def move_by_cmd_vel(self, path):
	return

    def move_by_move_base_grid_graph(self, path):
	pose = self.path_to_pose(path)
	for p in pose:
	    if self.exp_s.get_status != exp.move:
		return
	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = 'vmap'
	    goal.target_pose.header.stamp = rospy.Time.now()
	    goal.target_pose.pose = p
	    self.move_base.send_goal(goal)
	    try:
		finished_within_time = self.move_base.wait_for_result(rospy.Duration(30))
	    except rospy.ROSInterruptException:
		return
	    except rospy.exceptions.ROSException as e:
		rospy.logerr("Posi_publisher: Cannot achieve the goal!\nDetails: %s", e)
		return
	    if finished_within_time:
		rospy.logerr("Posi_publisher: Timed out achieving the goal!")
		return
	    else:
		if self.move_base.get_state() != GoalStatus.SUCCEEDED:
		    rospy.logerr("Posi_publisher: Cannot achieve the goal!")
		    return
	return

    def move_by_move_base_map_server(self, path):
	return

    def get_orientation(p1, p2):
	return atan2(p2[1] - p1[1], p2[0] - p1[0])

    def path_to_pose(self, path):
	dst_pose = []
	last_p = None
	for p in path:
	    if last_p is not None:
		dst_pose[-1].orientation = quaternion_from_euler(0, 0, self.get_orientation(last_p, p), axes = 'sxyz')
	    dst_path.append(Pose(Point(p[0], p[1], 0), Quaternion()))
	    last_p = p
	dst_pose[-1].orientation = dst_pose[-2].orientation
	return dst_pose

    def move_to(self, x, y):
	"Move to the point (x, y)"
	# TODO
	return

if __name__ == '__main__':
    navi = navi_nodes()
    navi.start()
