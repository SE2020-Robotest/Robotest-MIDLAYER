#!/usr/bin/python3
'''
Author: ou yang xu jian
Date: 2020-11-23 21:44:55
LastEditTime: 2020-11-26 10:20:52
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \se2020\communication\robot_client.py
'''
from __future__ import print_function

import grpc

import msg_pb2
import msg_pb2_grpc

receiverAddr = {
    "AR": {
        "IP": "localhost",
        "Port": 8888
    },
    "Ctrl": {
        "IP": "localhost",
        "Port": 8888
    },
    "Robot": {
        "IP": "localhost",
        "Port": 8888
    }
}


def getAddr(receiver):
    """
    get the receiver's ip address and port and bind together
    return the result
    """
    address = receiverAddr[receiver]["IP"] + ":" + \
        "{}".format(receiverAddr[receiver]["Port"])
    return address


def sendRBPosition(receiver, point, angle, velocity, timestamp):
    '''
    description: this function send robot position the the receiver.
    param {str} receiver
    param {list[posx, posy]} point
    param {float} angle
    param {list[vx, vy]} velocity
    param {int32} timestamp
    return {int} feedback response status
    '''
    address = getAddr(receiver)
    print("send robot position to " + receiver + ": " + address)
    channel = grpc.insecure_channel(address)
    stub = msg_pb2_grpc.MsgServicesStub(channel)
    PointPos = msg_pb2.Point(posx=point[0], posy=point[1])
    RBPos = msg_pb2.RBPosition(
        pos=PointPos,
        angle=angle,
        vx=velocity[0],
        vy=velocity[1],
        timestamp=timestamp
    )
    response = stub.RobotPosition(RBPos)
    print("Send Robot Position Feedback" + "{}".format(response.status))
    return response.status


def sendRBPath(receiver, robotpath, starttime=0, endtime=0):
    '''
    description: this function send the robot path to the receiver.
    param {str} receiver
    param {list[[posx, posy],...]} robotpath
    param {int} starttime: starttime isn't necessary
    param {int} endtime: endtime isn't necessary
    return {int} feedback response status
    '''
    address = getAddr(receiver)
    print("send robot path to " + receiver + ": " + address)
    channel = grpc.insecure_channel(address)
    stub = msg_pb2_grpc.MsgServicesStub(channel)
    path = msg_pb2.RBPath()
    for posx, posy in robotpath:
        point = path.pos.add()
        point.posx = posx
        point.posy = posy
    path.starttime = starttime
    path.endtime = endtime
    response = stub.RobotPath(path)
    print("Send Robot Path Feedback" + "{}".format(response.status))
    return response.status


def sendVoiceResult(receiver, voiceresult, timestamp):
    '''
    description: this function send the recognition result of voice file to the receiver.
    param {str} receiver: the message's receiver
    param {str} voiceresult: the recognition result of voice file
    param {int} timestamp
    return {int} feedback response status
    '''
    address = getAddr(receiver)
    print("send robot path to " + receiver + ": " + address)
    channel = grpc.insecure_channel(address)
    stub = msg_pb2_grpc.MsgServicesStub(channel)
    resultmsg = msg_pb2.VoiceStr(voice=voiceresult, timestamp=timestamp)
    response = stub.VoiceResult(resultmsg)
    print("Send Voice Recognition result Feedback" + "{}".format(response.status))
    return response.status


def sendRobotFinishedMsg(receiver):
    '''
    description: this function send the robot finished message to the receiver.
    param {str} receiver: the message's receiver
    return {int} feedback response status
    '''
    address = getAddr(receiver)
    print("send robot path to " + receiver + ": " + address)
    channel = grpc.insecure_channel(address)
    stub = msg_pb2_grpc.MsgServicesStub(channel)
    resultmsg = msg_pb2.Response(status=2)
    response = stub.RobotFinished(resultmsg)
    print("Send Robot Finished Feedback" + "{}".format(response.status))
    return response.status


if __name__ == "__main__":
    sendRBPosition("Ctrl", point=[3., 5.], angle=4.6,
                   velocity=[3.5, 9.5], timestamp=10)
    sendRBPath("Ctrl", robotpath=[[1., 2.], [3., 4.], [5., 6.]])
    sendRobotFinishedMsg("Ctrl")
    sendVoiceResult("Ctrl", voiceresult="Hello, GRPC", timestamp=10)
