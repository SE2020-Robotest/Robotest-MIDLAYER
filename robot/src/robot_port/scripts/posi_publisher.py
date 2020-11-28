#!/usr/bin/env python


import rospy
import trans
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
	if status == "experimenting":
	    try:
	        msg = trans.get_msg()
	        p = trans.get_pose(msg)
	    except rospy.ROSInterruptException:
		return
	    except rospy.exceptions.ROSException as e:
		rospy.logerr("Posi_publisher: Cannot get the position!\nDetails: %s", e)
		rospy.loginfo("Posi_publisher: Wait 10 seconds")
		rospy.sleep(10)
		continue
	    posi_pub.publish(msg.header.stamp.secs, p[0], p[1], p[2], p[3], p[4])
	rospy.sleep(5)

if __name__ == '__main__':
    pub()
