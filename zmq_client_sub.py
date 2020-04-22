import zmq
import sys
import zlib
import pickle


class Block(object):
    def __init__(self, position, status, final_destination):
        self.current_position = position
        self.status = status
        self.final_destination = final_destination

    def __str__(self):
        return str(self.__dict__)

class Division(object):
    def __init__(self):
        pass

    def get_locations_to_place_blocks(self):
        return [(4, 4, 4, "Top"),(5, 5, 5, "Top"),(6, 6, 6, "Top")]

class FerryBlocksStore(object):
    def __init__(self, blocks_to_remove, division):
        self.blocks_to_remove = blocks_to_remove
        self.division = division
        self.locations_to_place_blocks = division.get_locations_to_place_blocks()

    def __str__(self):
        return str(self.__dict__)

class Message:
    def __init__(self, message_id):
        self.message_id = message_id

class FerryBlocksMessage(Message):
    def __init__(self, blocks_to_move):
        super().__init__(message_id="FERRY")
        self.blocks_to_move = blocks_to_move

class BuildMessage(Message):
    def __init__(self):
        super().__init__(message_id="BUILD")

class WaitMessage(Message):
    def __init__(self):
        super().__init__(message_id="WAIT")

class MoveToPointMessage(Message):
    def __init__(self):
        super().__init__(message_id="MOVE")

class MessageWrapper:
    def __init__(self, topic, message):
        self.topic = topic
        self.message = message

# port = "5557"
# if len(sys.argv) > 1:
#     port = sys.argv[1]
#     int(port)

# if len(sys.argv) > 2:
#     port1 = sys.argv[2]
#     int(port1)

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collecting updates...")
# socket.connect(f"tcp://localhost:{port}")
# socket.connect("tcp://127.0.0.1:5558")
socket.bind("tcp://0.0.0.0:5559")


# if len(sys.argv) > 2:
#     socket.connect(f"tcp://localhost:{port1}")

topicfilter = b""
socket.setsockopt(zmq.SUBSCRIBE, topicfilter)

# Process 5 updates
total_value = 0
while True:
    [topic, message] = socket.recv_multipart()
    # print(topic)
    # print(message)
    message = zlib.decompress(message)
    message = pickle.loads(message)

    # print(message)
    print(message.topic)
    print(message.message)
    print("-"*20)
    print("\n")
    # print(message.message.message_id)
    # z = socket.recv_json()
    # p = zlib.decompress(z)
    # load = pickle.loads(p)
    # topic = load.topic
    # messagedata = topic.message
    # string = str(socket.recv())
    # topic, *messagedata = string.split()
    # total_value += int(messagedata)
    # print(topic, messagedata)
    # print(z)

