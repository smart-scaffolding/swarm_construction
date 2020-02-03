#!/usr/bin/env python

import time
import zmq
from zmq.eventloop import ioloop, zmqstream
import components.structure.config as config
# from queue import Queue
# import threading

from multiprocessing import Process, Queue


class HeartBeater():
    def __init__(self, robot_queue, period=1000):
        context = zmq.Context()
        pub = context.socket(zmq.PUB)

        heartbeat_connection_in = config.communication["heartbeat_connection_in"]
        heartbeat_connection_out = config.communication["heartbeat_connection_out"]

        pub.bind(heartbeat_connection_in)

        router = context.socket(zmq.ROUTER)
        router.bind(heartbeat_connection_out)
        # router.setsockopt(zmq.IDENTITY, b'ROBOT') # May want to enable this to control who sends heartbeats

        self.loop = ioloop.IOLoop().instance()
        outstream = zmqstream.ZMQStream(pub, self.loop)
        instream = zmqstream.ZMQStream(router, self.loop)

        self.period = period

        self.pingstream = outstream
        self.pongstream = instream
        self.pongstream.on_recv(self.handle_pong)

        self.hearts = set()
        self.responses = set()
        self.lifetime = 0
        self.tic = time.time()

        self.caller = ioloop.PeriodicCallback(self.beat, period, self.loop)
        self.caller.start()

        self.robot_queue = robot_queue

        self.loop.start()

    def beat(self):
        toc = time.time()
        self.lifetime += toc-self.tic
        self.tic = toc

        goodhearts = self.hearts.intersection(self.responses)
        heartfailures = self.hearts.difference(goodhearts)
        newhearts = self.responses.difference(goodhearts)

        if len(newhearts) > 0:
            self.handle_new_heart(newhearts)
        if len(heartfailures) > 0:
            self.handle_heart_failure(heartfailures)
        self.responses = set()

        print("%i beating hearts: %s"%(len(self.hearts),self.hearts))
        self.pingstream.send(str(self.lifetime).encode())

    def handle_new_heart(self, heart):
        while len(heart) > 0:
            new_heart = heart.pop()
            print(f"Detected new heart {str(new_heart)}")
            self.hearts.add(new_heart)
            self.robot_queue.put(new_heart)
            print("New heart added to queue")

    def handle_heart_failure(self, heart):
        while len(heart) > 0:
            print("HEART FAILURE")
            heart_failure = heart.pop()
            if heart_failure == b'SIMULATOR':
                print("The simulator has disconnected")
            elif "ROBOT" in str(heart_failure):
                print(f"A robot has been disconnected, {heart_failure}")
            else:
                print(f"Heart {str(heart_failure)} failed, no longer connected")
            self.hearts.remove(heart_failure)

    def handle_pong(self, msg):
        "if heart is beating"
        print("Handle pong")
        if msg[1] == str(self.lifetime).encode():
            self.responses.add(msg[0])
        else:
            print(f"Bad heartbeat (possibly old?): {msg[1]}")


def create_heartbeat_detector(robot_queue):
    hb = HeartBeater(robot_queue)


def start_hearbeat_detector(robot_queue):
    p = Process(target=create_heartbeat_detector, args=[robot_queue])
    p.start()
    return p




if __name__ == '__main__':
    # loop = ioloop.IOLoop()
    # context = zmq.Context()
    # pub = context.socket(zmq.PUB)
    # pub.bind('tcp://127.0.0.1:5555')
    # router = context.socket(zmq.ROUTER)
    # router.bind('tcp://127.0.0.1:5556')
    # # router.setsockopt(zmq.IDENTITY, b'ROBOT')
    #
    # outstream = zmqstream.ZMQStream(pub, loop)
    # instream = zmqstream.ZMQStream(router, loop)
    #
    # hb = HeartBeater(loop, outstream, instream, Queue())
    #
    # loop.start()
    queue = Queue()

    start_hearbeat_detector(queue)
    while True:
        print("Tick")
        if not queue.empty():
            print("Returning robot")
            print(queue.get())
        time.sleep(0.5)
