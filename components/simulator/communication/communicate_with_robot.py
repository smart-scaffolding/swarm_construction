import pickle
import threading
import zlib
from multiprocessing import Queue

import zmq
from logzero import logger

import components.simulator.config as Config


class SimulatorCommunicator(threading.Thread):
    """
    Used to listen for incoming messages to the simulator. This class inherits from the thread class and is run in a
    separate thread.

    """

    def __init__(self, robot_q, block_q, structure_q):
        super(SimulatorCommunicator, self).__init__()
        self.robot_actors = {}
        self.robot_q = robot_q
        self.stoprequest = threading.Event()
        self.block_q = block_q
        self.structure_q = structure_q

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.bind(Config.communication["receive_messages_port"])
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

    def run(self):
        while not self.stoprequest.isSet():
            try:
                [topic, message] = self.socket.recv_multipart()
                message = zlib.decompress(message)
                messagedata = pickle.loads(message)
                # logger.debug(f"[Worker thread]: {topic} {messagedata}")
                if "BLOCK" in str(topic.decode()):
                    # print(f"[Worker thread]: Got block message: {topic} -> {messagedata}")
                    self.block_q.put((topic, messagedata))
                if "ROBOT" in str(topic.decode()):
                    # print(f"[Worker thread]: Got robot message: {topic} -> {messagedata}")
                    if topic not in self.robot_actors:
                        self.robot_actors[topic] = Queue()
                        self.robot_actors[topic].put((topic, messagedata))
                        self.robot_q.put((topic, messagedata, self.robot_actors[topic]))
                    else:
                        self.robot_actors[topic].put((topic, messagedata))
                if "STRUCTURE" in str(topic.decode()):
                    self.structure_q.put((topic, messagedata))
                else:
                    continue
            except:
                logger.error(f"[Worker thread]: EXCEPTION")
                continue

    def join(self, timeout=None):
        self.stoprequest.set()
        super(SimulatorCommunicator, self).join(timeout)
