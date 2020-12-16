#!/usr/bin/python3
'''
Author: ou yang xu jian
Date: 2020-11-23 21:44:44
LastEditTime: 2020-12-04 22:53:00
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \se2020\communication\robot_server.py
'''
from threading import Lock
from concurrent import futures
import time

import grpc

import msg_pb2
import msg_pb2_grpc

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

Port = 8888


class RobotServicer(msg_pb2_grpc.MsgServicesServicer):

    def __init__(self, receiveMap, receiveCommand, receiveVoice, receiveDriveCommand):
        '''
        The following are callback function for handling the message
        '''
        self.receiveMap = receiveMap
        self.receiveCommand = receiveCommand
        self.receiveVoice = receiveVoice
        self.receiveDriveCommand = receiveDriveCommand
        return

    def ConfigMap(self, request, context):
        '''
        description: this function is the response function when
        the control site sends config map message to robot.
        param {*} self
        param {Map} request: the config map message
        param {*} context
        return {Response} feedback response
        TODO: post the custom message of getting config map
        '''
        # The following code prints the received message
        width = request.roomwidth
        height = request.roomheight
        blocks = request.blocks
        print("Room Width: {}".format(width))
        print("Room Heigth: {}".format(height))
        for block in blocks:
            print("Type: {}, W: {}, H: {}, Pos x: {}, Pos y: {}".format(
                block.type, block.w, block.h, block.pos.posx, block.pos.posy))

        # if the reqeust message goes wrong, please modify the status to 1
        if self.receiveMap is not None:
            # The Callback function
            return msg_pb2.Response(status=self.receiveMap(request, context))
        else:
            return msg_pb2.Response(status=0)

    def ControlCommand(self, request, context):
        '''
        description: this function is the response function when
        the control site send control command suck as start experiment
        or stop experiment.
        param {*} self
        param {ControlCmd} request: the ControlCmd message
        param {*} context
        return {Response} feedback response
        TODO: post the custom message of getting Control Command
        '''
        # The following code prints the received message
        # if the reqeust message goes wrong, please modify the status to 1
        if self.receiveCommand is not None:
            # The Callback function
            return msg_pb2.Response(status=self.receiveCommand(request, context))
        else:
            return msg_pb2.Response(status=0)

    def SendVoiceFile(self, request_iterator, context):
        '''
        description: this function is the response function when
        the AR site send voice file to the robot site.
        param {*} self
        param {VoiceFile} request: the bytes of voice file, which is str
        param {*} context
        return {Response} feedback response
        TODO: post the custom message of getting voice file
        '''
        # The following code save the voice file
        '''
        filename = "example.wav"
        with open(filename, "wb+") as file:
            for bytestr in request_iterator:
                file.write(bytestr)
                print(bytestr)
        '''
        # according to protocol buffer document, bytestr is a string.
        # bytestr need convert to byte buffer and then write into the files.
        # TODO: please implement the convert method.
        for bytestr in request_iterator:
            print(bytestr.file)
        # if the reqeust message goes wrong, please modify the status to 1
        if self.receiveVoice is not None:
            # The Callback function
            return msg_pb2.Response(status=self.receiveVoice(request_iterator, context))
        else:
            return msg_pb2.Response(status=0)

    def DriveRobot(self, response, context):
        '''
        description: this function is the response function when the control server site sends drive robot message
        param {*} self
        param {Drive} response
        param {*} context
        return {Response}
        '''
        return msg_pb2.Response(status=self.receiveDriveCommand(response, context))

    """
    def RobotPath(self, request, context):
        '''
        for test, the robot site doesn't need it
        '''
        points = request.pos
        for point in points:
            print("pos x {0}, pos y {1}".format(point.posx, point.posy))
        return msg_pb2.Response(status=0)

    def RobotFinished(self, request, context):
        '''
        for test, the robot site doesn't need it
        '''
        status = request.status
        print("finished")
        return msg_pb2.Response(status=0)
    """


_ROBOT_SERVER_RUNNING = True
lock = Lock()


def serve(servicer):
    global _ROBOT_SERVER_RUNNING
    _ROBOT_SERVER_RUNNING = True
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    msg_pb2_grpc.add_MsgServicesServicer_to_server(
        servicer, server
    )
    server.add_insecure_port('[::]:{}'.format(Port))
    server.start()
    try:
        while _ROBOT_SERVER_RUNNING:
            # time.sleep(_ONE_DAY_IN_SECONDS)
            pass
    except KeyboardInterrupt:
        server.stop(0)


def stop():
    global _ROBOT_SERVER_RUNNING
    with lock:
        _ROBOT_SERVER_RUNNING = False


if __name__ == '__main__':
    import _thread
    robotserver = RobotServicer()
    _thread.start_new_thread(serve, (robotserver,))
    # serve(robotserver)
    print("start server")
    time.sleep(5)
    stop()
    time.sleep(1)
