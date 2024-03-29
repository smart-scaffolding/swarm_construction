from __future__ import print_function

import pickle
import threading
import zlib
from queue import Queue

import vtk
import zmq

from .messages import MessageWrapper


class SendTopicToSimulatorThread(threading.Thread):
    def __init__(self, dir_q, socket_port, topics):
        super(SendTopicToSimulatorThread, self).__init__()

        self.dir_q = dir_q
        self.stoprequest = threading.Event()
        self.context = zmq.Context()
        self.socket = self.context.socket((zmq.PUB))
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
                print(
                    f"[SendTopicToSimulatorThread]: Sending communication to simulator -> {topic} {messagedata}"
                )
            # except:
            #     continue

    def join(self, timeout=None):
        self.stoprequest.set()
        super(SendTopicToSimulatorThread, self).join(timeout)


def vtk_named_colors(colors):
    """
    Returns a list of vtk colors
    :param colors: List of color names supported by vtk
    :return: A list of vtk colors
    """
    if type(colors) is not list:
        colors = [colors]
    colors_rgb = [0] * len(colors)
    for i in range(len(colors)):
        colors_rgb[i] = list(vtk.vtkNamedColors().GetColor3d(colors[i]))
    return colors_rgb


class SimulatorCommunication:
    def __init__(self, send_messages_socket, send_topics):
        self.send_messages_queue = Queue()
        self.send_messages_socket = send_messages_socket
        self.send_topics = send_topics

    def send_communication(self, message, topic=b"STRUCTURE"):
        self.send_messages_queue.put((topic, message))

    def initialize_communication_with_simulator(self):
        self.send_messages_thread = SendTopicToSimulatorThread(
            dir_q=self.send_messages_queue,
            socket_port=self.send_messages_socket,
            topics=self.send_topics,
        )
        pool = [self.send_messages_thread]
        for thread in pool:
            thread.daemon = True
            thread.start()
