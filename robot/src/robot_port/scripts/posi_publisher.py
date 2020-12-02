#!/usr/bin/env python


import rospy
from trans import trans
from status import rb
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
    posi_pub = rospy.Publisher('posi_pub', posi, queue_size = 10)
    rospy.init_node('posi_publisher', anonymous = False)

    rb_s = rb()
    tr = trans()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	status = rb_s.get_status()
	if status == rb.run:
	    try:
	        msg = tr.get_msg()
	        p = tr.get_pose(msg)
	    except rospy.ROSInterruptException:
		return
	    except rospy.exceptions.ROSException as e:
		rospy.logerr("Posi_publisher: Cannot get the position!\nDetails: %s", e)
		rospy.loginfo("Posi_publisher: Wait 10 seconds")
		rospy.sleep(10)
		continue
	    posi_pub.publish(msg.header.stamp.secs, p[0], p[1], p[2], p[3], p[4])
	    rate.sleep()
	else:
	    rospy.sleep(1)

if __name__ == '__main__':
    pub()
