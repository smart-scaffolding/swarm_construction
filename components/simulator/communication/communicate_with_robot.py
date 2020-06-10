import threading


class WorkerThread(threading.Thread):
    """

    """

    def __init__(self, dir_q, result_q, filter_q, socket, pipeline, block_q):
        super(WorkerThread, self).__init__()
        self.robot_actors = {}
        self.dir_q = dir_q
        self.result_q = result_q
        self.filter_q = filter_q
        self.stoprequest = threading.Event()
        self.socket = socket
        self.pipeline = pipeline
        self.block_q = block_q

    def run(self):
        """

        """
        while not self.stoprequest.isSet():
            try:
                [topic, message] = self.socket.recv_multipart()
                message = zlib.decompress(message)
                messagedata = pickle.loads(message)
                logger.debug(f"[Worker thread]: {topic} {messagedata}")
                if "BLOCK" in str(topic.decode()):
                    print(
                        f"[Worker thread]: Got block message: {topic} -> {messagedata}"
                    )
                    self.block_q.put((topic, messagedata))
                if "ROBOT" in str(topic.decode()):
                    if topic not in self.robot_actors:
                        self.robot_actors[topic] = Queue()
                        self.robot_actors[topic].put((topic, messagedata))
                        self.filter_q.put(
                            (topic, messagedata, self.robot_actors[topic])
                        )
                    else:
                        self.robot_actors[topic].put((topic, messagedata))
                else:
                    continue
            except:
                continue

    def join(self, timeout=None):
        """

        :param timeout:
        """
        self.stoprequest.set()
        super(WorkerThread, self).join(timeout)
