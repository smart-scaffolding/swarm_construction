from components.robot.communication import *
from components.simulator.communication.messages import *
import zlib
import pickle

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://127.0.0.1:5559")

while True:

    # TODO: TREVOR, set the topic and location of blocks here
    topic = None
    location = None

    messagedata = BlockLocationMessage(block_id=topic, location=location)
    message_obj = MessageWrapper(topic=topic, message=messagedata)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)
    print(f"{topic} {z}")

    socket.send_multipart([topic, z])
    time.sleep(3)
