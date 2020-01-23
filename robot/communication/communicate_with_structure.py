from __future__ import print_function
# import vtk
# from time import sleep
# from vtk import vtkCallbackCommand
# from multiprocessing import Process
from .messages import MessageWrapper
# import threading
import zmq
import sys
import math
import os, time
from queue import Queue
import threading
import zlib
import pickle
# import cPickle as pickle

class ReceiveTopicThread(threading.Thread):
    def __init__(self, result_q, socket_port, topics):
        super(ReceiveTopicThread, self).__init__()
        self.robot_actors = {}
        self.result_q = result_q
        self.stoprequest = threading.Event()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket_port = socket_port
        self.socket.connect(self.socket_port)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.topics = topics


    def run(self):
        while not self.stoprequest.isSet():
            """
            Uncommment the try except will developing, add back in for final. Otherwise, will not see errors in thread
            """
            # try:

            [topic, message] = self.socket.recv_multipart()
            message = zlib.decompress(message)
            messagedata = pickle.loads(message)

            print(f"[RecievedTopicThread]: Received communication from structure -> {topic} {messagedata}\n")
            if topic in self.topics:
                print(f"Message is in subscribed topics: {topic} {messagedata.message}")
                self.result_q.put((topic, messagedata.message))
            # except:
            #     continue

    def join(self, timeout=None):
        self.stoprequest.set()
        super(ReceiveTopicThread, self).join(timeout)

class SendTopicThread(threading.Thread):
    def __init__(self, dir_q, socket_port, topics):
        super(SendTopicThread, self).__init__()
        self.robot_actors = {}
        self.dir_q = dir_q
        self.stoprequest = threading.Event()
        self.context = zmq.Context()
        self.socket = self.context.socket((zmq.PUB))
        # self.socket.bind(socket_port)
        self.socket.connect(socket_port)
        self.socket_port = socket_port
        self.topics = topics


    def run(self):
        while not self.stoprequest.isSet():
           """
           Uncommment the try except will developing, add back in for final. Otherwise, will not see 
           errors in thread
           """
           # try:
           if not self.dir_q.empty():
                topic, messagedata = self.dir_q.get()
                message_obj = MessageWrapper(topic=topic, message=messagedata)

                message_pickle = pickle.dumps(message_obj, protocol=-1)
                messagedata_compressed = zlib.compress(message_pickle)

                self.socket.send_multipart([topic, messagedata_compressed])
                print(f"[SendTopicThread]: Sending communication to structure -> {topic} {messagedata}")
            # except:
            #     continue

    def join(self, timeout=None):
        self.stoprequest.set()
        super(SendTopicThread, self).join(timeout)

class StructureCommunication:
    def __init__(self, receive_messages_socket, send_messages_socket, send_topics, receive_topics):
        self.receive_messages_queue = Queue()
        self.send_messages_queue = Queue()
        self.receive_messages_socket = receive_messages_socket
        self.send_messages_socket = send_messages_socket
        self.send_topics = send_topics
        self.receive_topics = receive_topics

    def get_communication(self):
        messages = []
        while not self.receive_messages_queue.empty():
            message = self.receive_messages_queue.get()
            messages.append(message)
        print(f"[Structure Communicator]: Messages downloaded {messages}")
        return messages

    def send_communication(self, topic, message):
        self.send_messages_queue.put((topic, message))

    def initialize_communication_with_structure(self):
        self.receive_messages_thread = ReceiveTopicThread(result_q=self.receive_messages_queue,
                                                          socket_port=self.receive_messages_socket,
                                                          topics=self.receive_topics)
        self.send_messages_thread = SendTopicThread(dir_q=self.send_messages_queue,
                                                    socket_port=self.send_messages_socket, topics=self.send_topics)
        pool = [self.receive_messages_thread, self.send_messages_thread]
        for thread in pool:
            thread.start()