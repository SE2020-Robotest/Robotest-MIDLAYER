#!/usr/bin/env python


import thread
import rospy
from status import rb, exp
from enum_list import *
from std_msgs.msg import String
from robot_port.msg import posi
from robot_port.msg import path_ori
from robot_port.msg import path
from robot_port.msg import voice_cmd
from robot_port.msg import map_object
from robot_port.msg import vmap
from robot_port.msg import res

import rb_message.robot_server as msg_server
import rb_message.robot_client as msg_client

test_mode = True

class communicater:

    def __init__(self):

	# Members
	self.init_ok = True
	self.rb_s = rb()
	self.exp_s = exp()

        self.map_pub = rospy.Publisher('virtual_map', vmap, queue_size = 5)

        rospy.init_node('communicater', anonymous = False)

        rospy.Subscriber('posi_pub', posi, self.send_rb_posi)
        rospy.Subscriber('path_ori', path_ori, self.send_rb_path_ori)
        rospy.Subscriber('path', path, self.send_rb_path)
        rospy.Subscriber('voice_cmd', voice_cmd, self.send_rb_voice_cmd)
        rospy.Subscriber('response', res, self.send_response)

	status = self.rb_s.get_status()
	rospy.loginfo("Communicater: The robot_status now is %s.", status)

        recive_srv = msg_server.RobotServicer(self.recive_map, self.recive_command, self.recive_voice)
        try:
            thread.start_new_thread(msg_server.serve, (recive_srv, ))
	    rospy.loginfo("Communicater: Recieving service Initialized!")
        except:
	    rospy.logerr("Communicater: unable to start the reciving service thread!")
	    self.init_ok = False
	
        rospy.loginfo("Communicater: Communicate Node Initialized!")
	return

    def start(self):
	rospy.spin()

    def send_rb_posi(self, msg):
	'''
	posi:
	    int32 stamp
	    float64 x
	    float64 y
	    float64 vx
	    float64 vy
	    float64 angle
	'''
	if test_mode:
	    return
	try:
            msg_client.sendRBPosition("Ctrl", [msg.x, msg.y], msg.angle, [msg.vx, msg.vy], msg.stamp)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to Control port! Details:\n %s", e)
	'''
	try:
            msg_client.sendRBPosition("AR", [msg.x, msg.y], msg.angle, [msg.vx, msg.vy], msg.stamp)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to AR port! Details:\n %s", e)
	'''
	return

    def send_rb_path_ori(self, msg):
	'''
	path_ori:
	    int32 start_time
	    int32 end_time
	    point_2d[] p
	point_2d:
	    float64 x
	    float64 y
	'''
	if test_mode:
	    return
	path = []
	for p in msg.p:
	    path.append([p.x, p.y])
	try:
	    msg_client.sendRBPath("Ctrl", path, msg.start_time, msg.end_time)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to Control port! Details:\n %s", e)
	return

    def send_rb_path(self, msg):
        '''
	path:
	    point_2d[] p
	point_2d:
	    float64 x
	    float64 y
	'''
	if test_mode:
	    return
	path = []
	for p in msg.p:
	    path.append([p.x, p.y])
	try:
	    msg_client.sendRBPath("Ctrl", path)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to Control port! Details:\n %s", e)
	return

    def send_rb_voice_cmd(self, msg):
        '''
	voice_cmd:
	    int32 stamp
	    string cmd
	'''
	if test_mode:
	    return
	try:
	    msg_client.sendVoiceResult("Ctrl", msg.cmd, msg.stamp)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to Control port! Details:\n %s", e)
	try:
	    msg_client.sendVoiceResult("AR", msg.cmd, msg.stamp)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to AR port! Details:\n %s", e)
	return

    def send_response(self, msg):
        '''
	voice_cmd:
	    int32 stamp
	    string cmd
	'''
	if test_mode:
	    return
	try:
	    msg_client.sendResponseMsg("Ctrl", msg.status)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to Control port! Details:\n %s", e)
	try:
	    msg_client.sendResponseMsg("AR", msg.status)
	except Exception as e:
	    rospy.logerr("Communicater: Cannot connect to AR port! Details:\n %s", e)
	return

    def recive_map(self, request, context):
	"Handling message 'map' from control_port"
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
	status = self.rb_s.get_status()
	if status == rb.sleep:
	    rospy.logerr("Communicater: Cannot init the map. The Exp hasn't started yet!")
	    return 0
	elif status != rb.init:
	    rospy.logerr("Communicater: Cannot init the map. There is an Exp running now!")
	    return 0
	obj = []
	for block in request.blocks:
	    obj.append(map_object(block.type, block.pos.posx, block.pos.posy, block.w, block.h))
	self.map_pub.publish(request.roomwidth, request.roomheight, obj) # Pubilsh the map to topic 'virtual_map'
        return 0

    def recive_command(self, request, context):
	status = self.rb_s.get_status()
	cmd = request.cmd
	if cmd == START: # start the exp
	    if status == rb.sleep:
		self.rb_s.Start()
	    else:
		rospy.logerr("Communicater: Cannot start the Exp. There is an Exp running now!")
	elif cmd == STOP: # stop the exp
	    if status != rb.sleep:
		self.exp_s.wait()
		self.rb_s.Stop()
	    else:
		rospy.logwarn("Communicater: Cannot stop the Exp. The Exp hasn't started yet!")
	elif cmd == CONNECT:
	    pass # build connect
	else:
	    return 1
        return 0

    def recive_voice(self, request_iterator, context):
	
        return 0

        


if __name__ == '__main__':
    try:
	comm = communicater()
	if comm.init_ok:
	    comm.start()
    except rospy.ROSInterruptException:
	pass
