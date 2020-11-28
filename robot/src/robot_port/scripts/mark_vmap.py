#!/usr/bin/env python


import rospy
import geometry_msgs.msg
import tf

class vmap_coordinate:
    def __init__(self):
	rospy.init_node('vmap_br', anonymous = False)
	self.t = geometry_msgs.msg.TransformStamped()
	return

    def mark(self):
        listener = tf.TransformListener()
        listener.waitForTransform("/base_link", "/odom", rospy.Time(0),rospy.Duration(4.0))
        [trans,rot] = listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = 'odom'
	self.t.header.stamp = rospy.Time.now()
        self.t.child_frame_id = 'vmap'
        self.t.transform.translation.x = trans[0]
        self.t.transform.translation.y = trans[1]
        self.t.transform.translation.z = trans[2]
        self.t.transform.rotation.x = rot[0]
        self.t.transform.rotation.y = rot[1]
        self.t.transform.rotation.z = rot[2]
        self.t.transform.rotation.w = rot[3]

    def start(self):
	m = tf.TransformBroadcaster()
	self.mark()
	rate = rospy.Rate(1)
    	while not rospy.is_shutdown():
	    self.t.header.stamp = rospy.Time.now()
	    try:
                m.sendTransformMessage(self.t)
	    except rospy.ROSInterruptException:
		break
            rate.sleep()

if __name__ == '__main__':  
    vmap = vmap_coordinate()
    vmap.start()
