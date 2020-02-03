from components.robot.common.states import Block, Division
from components.robot.communication import *
import zlib
import pickle
from random import choice

# port = "5556"
port = "5557"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)

context = zmq.Context()
socket = context.socket(zmq.PUB)
# socket.bind(f"tcp://*:{port}")
# socket.bind("tcp://127.0.0.1:5559")
socket.connect("tcp://127.0.0.1:5559")


while True:
    # topic = random.randrange(10001,10010)
    # topic = 10001
    topics = [b"ROBOT_1", b"ROBOT_2", b"ROBOT_3"]
    # topics = [b"ROBOT_1"]
    topic = choice(topics)
    # topic = b"ROBOT_1"
    # messagedata =

    blocks_to_place = [Block(position=(0, 0, 0, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
                       Block(position=(1, 1, 1, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
                       Block(position=(2, 2, 2, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top"))]

    division = Division()

    # move_store = MoveBlocksStore(blocks_to_remove=blocks_to_place, division=division)
    # messagedata = BuildMessage(blocks_to_move=move_store)


    messagedata = MoveToPointMessage(destination=(7, 7, 7, "Bottom"))

    message_obj = MessageWrapper(topic=topic, message=messagedata)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)
    print(f"{topic} {z}")
    # socket.send_string(f"{topic} {messagedata}")
    socket.send_multipart([topic, z])
    time.sleep(3)

    # messagedata = WaitMessage()
    #
    # message_obj = MessageWrapper(topic=topic, message=messagedata)
    # # message_obj = "{'test': 'test'}"
    # # message_json = json.dumps(message_obj)
    # p = pickle.dumps(message_obj, protocol=-1)
    #
    # z = zlib.compress(p)
    # # messagedata ='HI'
    # print(f"{topic} {z}")
    # # socket.send_string(f"{topic} {messagedata}")
    # socket.send_multipart([topic, z])
    # time.sleep(10)

    # move_store = MoveBlocksStore(blocks_to_remove=blocks_to_place, division=division)
    # messagedata = BuildMessage(blocks_to_move=move_store)
    #
    # message_obj = MessageWrapper(topic=topic, message=messagedata)
    # p = pickle.dumps(message_obj, protocol=-1)
    # z = zlib.compress(p)
    # print(f"{topic} {z}")
    # # socket.send_string(f"{topic} {messagedata}")
    # socket.send_multipart([topic, z])
    # time.sleep(10)