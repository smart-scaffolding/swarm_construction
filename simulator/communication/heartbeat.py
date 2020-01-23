#!/usr/bin/env python

import time
import numpy
import zmq
from zmq import devices

ctx = zmq.Context()

ID = b'SIMULATOR'

dev = devices.ThreadDevice(zmq.FORWARDER, zmq.SUB, zmq.DEALER)
dev.setsockopt_in(zmq.SUBSCRIBE, b"")
dev.setsockopt_out(zmq.IDENTITY, ID)
dev.connect_in('tcp://127.0.0.1:5555')
dev.connect_out('tcp://127.0.0.1:5556')
dev.start()