#!/usr/bin/env python


import rospy
import grid_graph as graph
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import path
from robot_port.msg import point_2d

'''
The origin point is on the bottom-left, and unit is cm

i          y
|          |
|       &  |
|_____ j   |_____ x

This is the corresbonding relation between grid_graph and virtual_map
'''

eps = 0.0001 # The tollerance

class navi_nodes:
    def __init__(self):
	self.map_loaded = False
	self.g = None
	self.move_cmd = Twist()
	self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
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
	rb_status = rospy.get_param("robot_status")
	if rb_status != 'exp_starting':
	    return
	self.g = graph.grid_graph(msg.w, msg.h)
	
	for obj in msg.obj:
	    if obj.type:
		self.g.add_circle(obj.x, obj.y, obj.w)
	    else:
		self.g.add_rect(obj.x, obj.y, obj.w, obj.h)
	self.g.finished()
	rospy.set_param("robot_status", "Experimenting")
	return

    def recieve_path(self, msg):
	"Recieve the path, correct them and move along this path. Finally, send the corrected path to Control port"
	rb_status = rospy.get_param("robot_status")
	exp_status = rospy.get_param("exp_status")
	if rb_status != 'experimenting' or exp_status != 'recognite_cmd':
	    return
	rospy.set_param("exp_status", "moving")
	
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
	
	"Check the status"
	rb_status = rospy.get_param("robot_status")
	exp_status = rospy.get_param("exp_status")
	if rb_status != 'experimenting' or exp_status != 'recognite_cmd':
	    return
	rospy.set_param("exp_status", "moving") 			# (Important!) Change the status

	"Correct the point"
	qx = msg.x # destination
	qy = msg.y # destination
	qj = int(qx)
	qi = int(qy)
	if not self.g.is_empt(qi, qj):
	    [ti, tj] = self.g.correct_point(qi, qj) # correct the destination
	    rospy.logerr("Navi: The destination is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]", qi, qj, ti, tj)
	    qy = qi = ti
	    qx = qj = tj
	"Get the Robot's position"
	try:
	    posi_msg = rospy.wait_for_message('/odom', Odometry, timeout = 5)
	except rospy.ROSInterruptException:
	    return
	except:
	    rospy.logerr("Navi: Cannot get the Robot's position!")
	    rospy.set_param("exp_status", "waiting") 			# (Important!) Change the status
	    return
	px = posi_msg.pose.pose.position.x # origin
	py = posi_msg.pose.pose.position.y # origin
	pj = int(px)
	pi = int(py)
	[pi, pj] = self.g.correct_point(pi, pj) # correct the origin

	"Find path"
	path = self.g.find_path(pi, pj, qi, qj)
	if path is None:
	    rospy.logerr("Navi: The destination is not accessable!")
	    rospy.set_param("exp_status", "waiting") 			# (Important!) Change the status
	    return

	"Send path message"	
	path_msg = []
	path_mag.p.append(point_2d(px, py)) # origin
	for p in path:
	    path_msg.append(point_2d(p[0]*self.g.delta, p[1]*self.g.delta))
	path_mag.append(point_2d(qx, qy)) # destination
	self.path_pub.publish(path_msg)

	"Move to the destination"
	for p in path:
	    self.move_to(p[0], p[1])
	self.move_to(qx, qy)
	rospy.loginfo("Navi: Arrive at destination!")
	rospy.set_param("exp_status", "waiting") 			# (Important!) Change the status
	
	return

    def move_to(self, x, y):
	"Move to the point (x, y)"
	
	return

if __name__ == '__main__':
    navi = navi_nodes()
    navi.start()
