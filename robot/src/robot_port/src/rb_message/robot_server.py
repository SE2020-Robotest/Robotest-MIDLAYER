#!/usr/bin/python3
'''
Author: ou yang xu jian
Date: 2020-11-23 21:44:44
LastEditTime: 2020-11-27 12:32:10
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \se2020\communication\robot_server.py
'''
from concurrent import futures
import time

import grpc

import msg_pb2
import msg_pb2_grpc

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

Port = 8888


class RobotServicer(msg_pb2_grpc.MsgServicesServicer):

    def __init__(self, reciveMap, reciveCommand, reciveVoice):
        '''
        The following are callback function for handling the message
        '''
        self.reciveMap = reciveMap
        self.reciveCommand = reciveCommand
        self.reciveVoice = reciveVoice
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
        if self.reciveMap is not None:
            return msg_pb2.Response(status=self.reciveMap(request, context)) # The Callback function
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
        cmd = request.cmd
        if cmd == 0:
            print("Start the Experiment")
        elif cmd == 1:
            print("Stop the Experiment")
        elif cmd == 2:
            print("build connect")

        # if the reqeust message goes wrong, please modify the status to 1
        if self.reciveCommand is not None:
            return msg_pb2.Response(status=self.reciveCommand(request, context)) # The Callback function
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
        if self.reciveVoice is not None:
            return msg_pb2.Response(status=self.reciveVoice(request_iterator, context)) # The Callback function
        else:
            return msg_pb2.Response(status=0)

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


def serve(servicer):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    msg_pb2_grpc.add_MsgServicesServicer_to_server(
        servicer, server
    )
    server.add_insecure_port('[::]:{}'.format(Port))
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == '__main__':
    serve(RobotServicer())
