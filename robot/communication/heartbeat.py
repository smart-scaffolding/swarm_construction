#!/usr/bin/env python

import time
import numpy
import zmq
from zmq import devices
from random import randint
import sys
# from zmq.utils.z85 import encode

def start_heartbeat(id, connection_in='tcp://127.0.0.1:5555', connection_out='tcp://127.0.0.1:5556'):
    ctx = zmq.Context()

    ID = id
    # if len(sys.argv) > 1:
    #     ID = sys.argv[1].encode('UTF-8')
        # str(ID).encode('UTF-8')

    dev = devices.ThreadDevice(zmq.FORWARDER, zmq.SUB, zmq.DEALER)
    dev.setsockopt_in(zmq.SUBSCRIBE, b"")
    dev.setsockopt_out(zmq.IDENTITY, ID)
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



