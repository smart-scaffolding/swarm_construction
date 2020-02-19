#!/usr/bin/env python

import time
import numpy
import zmq
from zmq import devices
from random import randint
import pickle
import zlib
from components.robot.communication.messages import HeartBeat
import py_trees
import sys
# from zmq.utils.z85 import encode





def start_heartbeat(id, connection_in='tcp://127.0.0.1:5555', connection_out='tcp://127.0.0.1:5556'):
    ctx = zmq.Context()

    ID = id
    # if len(sys.argv) > 1:
    #     ID = sys.argv[1].encode('UTF-8')
        # str(ID).encode('UTF-8')
    writer = py_trees.blackboard.Client(name="Writer")
    writer.register_key(key="state/current_position", access=py_trees.common.Access.READ)
    block_face = writer.get("state/current_position")
    position = (block_face.xPos, block_face.yPos, block_face.zPos)
    message_obj = HeartBeat(id=ID, position=position)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)

    dev = devices.ThreadDevice(zmq.FORWARDER, zmq.SUB, zmq.DEALER)

    dev.setsockopt_in(zmq.SUBSCRIBE, b"")
    dev.setsockopt_out(zmq.IDENTITY, z)
    dev.connect_in(connection_in)
    dev.connect_out(connection_out)
    dev.start()


if __name__ == '__main__':
    start_heartbeat(id=b'ROBOT' + str(randint(1, 100)).encode('UTF-8'))
    # wait for connections
    time.sleep(1)

    A = numpy.random.random((2 ** 11, 2 ** 11))
    print("starting blocking loop")
    while True:
        tic = time.time()
        numpy.dot(A, A.transpose())
        print("blocked for %.3f s" % (time.time() - tic))



