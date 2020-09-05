from components.robot.common.states import Block, Division
from components.robot.communication import *
from components.simulator.communication.messages import *
import zlib
import pickle
from random import choice, randint
import numpy as np
import zmq
import time
# # port = "5556"
# port = "5557"
# if len(sys.argv) > 1:
#     port =  sys.argv[1]
#     int(port)

context = zmq.Context()
socket = context.socket(zmq.PUB)
# socket.bind(f"tcp://*:{port}")
# socket.bind("tcp://127.0.0.1:5559")
socket.connect("tcp://127.0.0.1:5559")
#

while True:
    # topic = random.randrange(10001,10010)
    # topic = 10001
    # topics = [b"ROBOT_1", b"ROBOT_2", b"ROBOT_3"]
    topics = [b"ROBOT_1"]
    # topics = [b"BLOCK_1", b"BLOCK_2", b"BLOCK_3"]
    topic = choice(topics)
    # topic = b"BLOCK_" + str(randint(0, 10)).encode()
    # topic = b"ROBOT_1"
    # messagedata =

    # locations = [(0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 1)]
    location = (randint(0, 8), randint(0, 8), 1)
    blocks_to_place = [Block(position=(0, 0, 0, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
                       Block(position=(1, 1, 1, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
                       Block(position=(2, 2, 2, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top"))]

    # division = Division()

    # move_store = MoveBlocksStore(blocks_to_remove=blocks_to_place, division=division)
    # messagedata = BuildMessage(blocks_to_move=move_store)

    # messagedata = MoveToPointMessage(destination=(7, 7, 7, "Bottom"))

    base = np.matrix([[1, 0, 0, 0.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])
    base1 = np.matrix([[1, 0, 0, 1.5],
                       [0, 1, 0, 0.5],
                       [0, 0, 1, 1.],
                       [0, 0, 0, 1]])
    base2 = np.matrix([[1, 0, 0, 1.5],
                       [0, 1, 0, 1.5],
                       [0, 0, 1, 1.],
                       [0, 0, 0, 1]])
    base3 = np.matrix([[1, 0, 0, 4.5],
                       [0, 1, 0, 0.5],
                       [0, 0, 1, 1.],
                       [0, 0, 0, 1]])
    base4 = np.matrix([[1, 0, 0, 4.5],
                       [0, 1, 0, 1.5],
                       [0, 0, 1, 1.],
                       [0, 0, 0, 1]])
    base5 = np.matrix([[1, 0, 0, 5.5],
                       [0, 1, 0, 0.5],
                       [0, 0, 1, 1.],
                       [0, 0, 0, 1]])

    base = choice([base, base1, base2, base3, base4, base5])
    base = base1
    # base = base1
    #0, 2, 3, 4, 6
    trajectory1 = np.array([[0, 0, 0, 0, 0, 0, 0]])
    trajectory2 = np.array([[np.pi / 2, 0, np.pi / 2, np.pi / 2, np.pi / 2, 0, np.pi]])
    trajectory2 = np.array([[-1.57079633, 0., 2.1261001, -2.01502061, -1.68187582, 0., 0.]])
    trajectory2 = np.array([[-1.57079633, 0., 1.08612036, -2.17062641, -0.48629028, 0., 0.]])
    # trajectory2 = np.array([[np.pi / 2, 0, 0, 0, 0, 0, 0]])  # j1 Good
    # trajectory2 = np.array([[0, 0, np.pi / 2, 0, 0, 0, 0]])  # j2 Good 
    # trajectory2 = np.array([[0, 0, 0, np.pi / 2, 0, 0, 0]])  # j3 Good
    # trajectory2 = np.array([[0, 0, 0, 0, np.pi / 2, 0, 0]])  # j4 Good
    # trajectory2 = np.array([[0, 0, 0, 0, 0, 0, np.pi]])

    trajectory3 = np.array([[0, 0, 0, np.pi / 2, 0, 0]])
    trajectory4 = np.array([[np.pi, 0, np.pi / 2, np.pi / 2, 0, 0]])
    trajectory5 = np.array([[2 * np.pi, np.pi / 2, 0, 0, np.pi / 2, 0]])

    trajectory_choice = choice([trajectory2, trajectory3, trajectory4, trajectory5])
    trajectories = np.linspace(trajectory1, trajectory1, 60)
    for i in trajectories:

        print(f"TRAJ: {i.shape}")
        # trajectory_choice = choice([trajectory1, trajectory2, trajectory3, trajectory4, trajectory5])
        messagedata = AnimationUpdateMessage(robot_base=base, trajectory=i, debug_text="", block_on_ee=None)
        message_obj = MessageWrapper(topic=topic, message=messagedata)
        p = pickle.dumps(message_obj, protocol=-1)
        z = zlib.compress(p)
        print(f"{topic} {z}")
        # socket.send_string(f"{topic} {messagedata}")
        socket.send_multipart([topic, z])
        # time.sleep(3)

    time.sleep(1)
    # messagedata = WaitMessage()
    #
    # message_obj = MessageWrapper(topic=topic, message=messagedata)
    # # message_obj = "{'original': 'original'}"
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


# context = zmq.Context()
# socket = context.socket(zmq.PUB)
# socket.connect("tcp://127.0.0.1:5559")
#
# def send_to_simulator(base, trajectory, topic=b"ROBOT_1"):
#     messagedata = AnimationUpdateMessage(robot_base=base, trajectory=trajectory)
#     message_obj = MessageWrapper(topic=topic, message=messagedata)
#     p = pickle.dumps(message_obj, protocol=-1)
#     z = zlib.compress(p)
#     print(f"{topic} {z}")
#     socket.send_multipart([topic, z])

# import json

# import zmq

# ports = ["tcp://127.0.0.1:9998", "tcp://127.0.0.1:9999"]

# context = zmq.Context()
# # print("Connecting to machine...")

# sockets = []
# for index in range(len(ports)):
#     context = zmq.Context()
#     print("Connecting to machine...")
#     socket = context.socket(zmq.DEALER)
#     socket.setsockopt(zmq.LINGER, 0)
#     socket.bind("%s" % ports[index])
#     print("Successfully connected to machine %s" % ports[index])
#     sockets.append(socket)

# # socket[0].setsockopt(zmq.LINGER, 0)
# # for index in range(len(sockets)):

# # socket.connect("%s" % port)

# for request in range(len(ports)):
#     print("Sending request ", request, "...")
#     print(sockets[request])

#     sockets[request].send_string("", zmq.SNDMORE)  # delimiter
#     sockets[request].send_string("Sensor Data")  # actual message
#     print("Request sent")

#     # use poll for timeouts:
#     poller = zmq.Poller()
#     poller.register(sockets[request], zmq.POLLIN)

#     socks = dict(poller.poll(5 * 1000))
#     print(socks)

#     if sockets[request] in socks:
#         try:
#             sockets[request].recv()  # discard delimiter
#             msg_json = sockets[request].recv()  # actual message
#             sens = json.loads(msg_json)
#             response = "Sensor: %s :: Data: %s :: Client: %s" % (
#                 sens["sensor"],
#                 sens["data"],
#                 sens["client"],
#             )
#             print("Received reply ", request, "[", response, "]")
#         except IOError:
#             print("Could not connect to machine")
#     else:
#         print("Machine did not respond")
