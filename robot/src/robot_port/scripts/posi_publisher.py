#!/usr/bin/env python


import rospy
from robot_port.msg import posi


def pub():
    posi_pub = rospy.Publisher('send_rb_posi', posi, queue_size = 10)
    rospy.init_node('posi_publisher', anonymous = False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	status = rospy.get_param("robot_status")
	if status == "experimenting":
	    pass
