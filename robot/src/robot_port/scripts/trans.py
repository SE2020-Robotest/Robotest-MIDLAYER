#!/usr/bin/env python


import rospy
import tf
from math import cos, sin, pi
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
from robot_port.msg import posi

def rotate(x, y, theta):
    px = x*cos(theta) - y*sin(theta)
    py = x*sin(theta) + y*cos(theta)
    return [px, py]

def normalize_angle(theta):
    if theta > pi:
	k = int(theta / (2*pi))
	theta -= (k + 1)*pi
    elif theta < -pi:
	k = int(-theta / (2*pi))
	theta += (k + 1)*pi
    return theta

def quat_to_angle(quat):
    (r, p, y) = tf.transformations.euler_from_quaternion(quat)
    return y

def get_msg():
    try:
	msg = rospy.wait_for_message('/odom', Odometry, timeout = 5)
	rospy.loginfo("Got the posi!")
    except rospy.ROSInterruptException:
	return
    return msg

def _point_a_to_b(frame_a, frame_b, x, y):
    "Transform the coordinates of a point in frame_a into frame_b."
    listener = tf.TransformListener()
    listener.waitForTransform("/" + frame_b, "/" + frame_a, rospy.Time(0),rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/" + frame_b, "/" + frame_a, rospy.Time(0))
    theta = quat_to_angle(rot)
    [x, y] = rotate(x, y, theta)
    x += trans[0]
    y += trans[1]
    return [x, y]

def _vector_a_to_b(frame_a, frame_b, x, y):
    "Transform the coordinates of a vector in frame_a into frame_b. Vector just need to be rotated."
    listener = tf.TransformListener()
    listener.waitForTransform("/" + frame_b, "/" + frame_a, rospy.Time(0),rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/" + frame_b, "/" + frame_a, rospy.Time(0))
    theta = quat_to_angle(rot)
    return rotate(x, y, theta)

def _angle_a_to_b(frame_a, frame_b, theta):
    "Transform the coordinates of a vector in frame_a into frame_b. Vector just need to be rotated."
    listener = tf.TransformListener()
    listener.waitForTransform("/" + frame_b, "/" + frame_a, rospy.Time(0),rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/" + frame_b, "/" + frame_a, rospy.Time(0))
    theta += quat_to_angle(rot)
    return normalize_angle(theta)

def get_posi(msg = None):
    if msg is None:
        msg = get_msg()
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    [px, py] = point_world_to_vmap(px, py)
    return [100 * px, 100 * py]

def get_angle(msg = None):
    if msg is None:
        msg = get_msg()
    theta = quat_to_angle([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    return angle_world_to_vmap(theta)

def get_velocity(msg = None):
    if msg is None:
        msg = get_msg()
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    return vector_robot_to_vmap(100 * vx, 100 * vy)

def get_pose(msg = None):
    if msg is None:
        msg = get_msg()
    [px, py] = get_posi(msg)
    theta = get_angle(msg)
    [vx, vy] = get_velocity(msg)
    return [px, py, vx, vy, theta]

def point_world_to_robot(x, y):
    return _point_a_to_b("odom", "base_link", x, y)

def vector_world_to_robot(x, y):
    return _vector_a_to_b("odom", "base_link", x, y)

def point_robot_to_world(x, y):
    return _point_a_to_b("base_link", "odom", x, y)

def vector_robot_to_world(x, y):
    return _vector_a_to_b("base_link", "odom", x, y)

def point_vmap_to_robot(x, y):
    return _point_a_to_b("vmap", "base_link", x, y)

def vector_vmap_to_robot(x, y):
    return _vector_a_to_b("vmap", "base_link", x, y)

def point_robot_to_vmap(x, y):
    return _point_a_to_b("base_link", "vmap", x, y)

def vector_robot_to_vmap(x, y):
    return _vector_a_to_b("base_link", "vmap", x, y)

def point_vmap_to_world(x, y):
    return _point_a_to_b("vmap", "odom", x, y)

def vector_vmap_to_world(x, y):
    return _vector_a_to_b("vmap", "odom", x, y)

def point_world_to_vmap(x, y):
    return _point_a_to_b("odom", "vmap", x, y)

def vector_world_to_vmap(x, y):
    return _vector_a_to_b("odom", "vmap", x, y)

def angle_world_to_vmap(theta):
    return _angle_a_to_b("odom", "vmap", theta)

