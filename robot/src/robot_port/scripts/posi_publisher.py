#!/usr/bin/env python


import rospy
from tf.transformations import euler_from_quaternion  
import math
from nav_msgs.msg import Odometry
from robot_port.msg import posi

'''
posi:
    int32 stamp
    float64 x
    float64 y
    float64 vx
    float64 vy
    float64 angle
'''

def pub():
    posi_pub = rospy.Publisher('send_rb_posi', posi, queue_size = 10)
    rospy.init_node('posi_publisher', anonymous = False)

    while not rospy.is_shutdown():
	status = rospy.get_param("robot_status")
	try:
	    msg = rospy.wait_for_message('/odom', Odometry)
	    rospy.loginfo("Got the posi!")
	except rospy.ROSInterruptException:
	    break
	if status == "no_exp":
	    (r, p, y) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
								msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	    
	    posi_pub.publish(msg.header.stamp.secs, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x, \
			    msg.twist.twist.linear.y, y)
	rospy.sleep(1)

if __name__ == '__main__':
    pub()
