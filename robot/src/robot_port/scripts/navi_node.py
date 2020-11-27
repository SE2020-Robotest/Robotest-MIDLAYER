#!/usr/bin/env python


import rospy
from grid_graph import grid_graph as graph
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import path
from robot_port.msg import point_2d


class navi_nodes:
    def __init__(self):
	self.map_loaded = False
	self.g = None
	self.path_pub = rospy.Publisher('send_rb_path', vmap, queue_size = 10)

	rospy.init_node('navi_nodes', anonymous = False)
        rospy.loginfo("Navigation Nodes Initialized!")

	rospy.Subscriber('virtual_map', vmap, self.load_map)
	rospy.Subscriber('path_message', path, self.recieve_path)
	rospy.Subscriber('dst_message', point_2d, self.recieve_dst)
	return

    def start(self):
	rospy.spin()
	return

    def load_map(self, msg):
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
	self.g = graph(msg.w, msg.h)
	for obj in msg.obj:
	    if obj.type:
		self.g.add_circle(obj.x, obj.y, obj.w)
	    else:
		self.g.add_rect(obj.x, obj.y, obj.w, obj.h)
	self.g.finished()
	rospy.set_param("robot_status", "Experimenting")
	return

    def recieve_path(self, msg):
	rb_status = rospy.get_param("robot_status")
	exp_status = rospy.get_param("exp_status")
	if rb_status != 'experimenting' or exp_status != 'recognite_cmd':
	    return
	rospy.set_param("exp_status", "moving")
	return

    def recieve_dst(self, msg):
	rb_status = rospy.get_param("robot_status")
	exp_status = rospy.get_param("exp_status")
	if rb_status != 'experimenting' or exp_status != 'recognite_cmd':
	    return
	rospy.set_param("exp_status", "moving")
	qx = msg.x
	qy = msg.y
	qi = int(qx)
	qj = int(qy)
	if not self.g.is_empt(qi, qj):
	    [ti, tj] = self.g.correct_point(qi, qj)
	    rospy.logerr("Navi: The destination is illegal!\nAutomatically correct it from [%s, %s] to [%s, %s]", qi, qj, ti, tj)
	    qi = ti
	    qj = tj
	try:
	    posi_msg = rospy.wait_for_message('/odom', Odometry, timeout = 5)
	except rospy.ROSInterruptException:
	    return
	except:
	    rospy.logerr("Navi: Cannot get the Robot's position!")
	    rospy.set_param("exp_status", "waiting")
	    return
	px = posi_msg.pose.pose.position.x
	py = posi_msg.pose.pose.position.y
	pi = int(px)
	pj = int(py)
	[pi, pj] = self.g.correct_point(pi, pj)
	# TODO
	return

if __name__ == '__main__':
    navi = navi_nodes()
    navi.start()
