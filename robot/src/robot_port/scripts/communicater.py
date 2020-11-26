#!/usr/bin/env python


import thread
import rospy

from std_msgs.msg import String
from robot_port.msg import posi
from robot_port.msg import path_ori
from robot_port.msg import path
from robot_port.msg import voice_cmd
from robot_port.msg import map_object
from robot_port.msg import vmap

import rb_message.robot_server as msg_server
import rb_message.robot_client as msg_client

class communicater:

    def send_rb_posi(self, msg):
        msg.x

    def send_rb_path_ori(self, msg):
        msg.start_time

    def send_rb_path(self, msg):
        msg.p

    def send_rb_voice_cmd(self, msg):
        msg.cmd

    def recive_map(self, request, context):
        pass

    def recive_command(self, request, context):
        pass

    def recive_voice(self, request_iterator, context):
        pass

    def __init__(self):
        self.map_pub = rospy.Publisher('init', vmap, queue_size = 2)

        rospy.init_node('communicater', anonymous = False)
	'''
        rospy.Subscriber('send_rb_posi', posi, send_rb_posi)
        rospy.Subscriber('send_rb_path_ori', path_ori, send_rb_path_ori)
        rospy.Subscriber('send_rb_path', path, send_rb_path)
        rospy.Subscriber('send_rb_voice_cmd', voice_cmd, send_rb_voice_cmd)
	'''
        print("Nodes Initialized!")

        recive_srv = msg_server.RobotServicer(self.recive_map, self.recive_command, self.recive_voice)
        try:
            thread.start_new_thread(msg_server.serve, (recive_srv, ))
	    print("Services Initialized!")
        except:
	    rospy.logerr("Error: unable to start the reciving service thread!")
	return
        


if __name__ == '__main__':
    try:
	comm = communicater()
        rate = rospy.Rate(10)
	while not rospy.is_shutdown():
	    rate.sleep()
        thread.exit()
    except rospy.ROSInterruptException:
	pass
