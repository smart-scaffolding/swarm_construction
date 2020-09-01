from components.robot.communication.messages import *

# from components.simulator.communication.messages import *
import zlib
import pickle
import serial
from uuid import uuid4
import zmq
import time

"""
Run command in terminal:
screen /dev/cu.usbmodem14101 115200

screen [PORT] [BAUD]

"""

port = "/dev/cu.usbmodem14201"  # change to serial port connected to homeblock arduino
ser = serial.Serial(port, 115200, timeout=None)


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://127.0.0.1:5559")

start = False
blocks = {}


while True:

    # TODONE: TREVOR, set the topic and location of blocks here
    line = str(ser.readline())[2:-5]

    if line == "start":
        start = True
    elif start and line != "" and line != "TIMEOUT!":
        line = line.split(",")
        print(line)
        try:
            x = line[0]
            y = line[1]
            z = line[2]
            mode = line[3]
            print(x, y, z)
            print(mode)

            location = (float(x), float(y), float(z))
            status = str(mode)

            if location in blocks:
                topic = blocks[location]
            else:
                topic = b"BLOCK_" + str(uuid4()).encode()
                blocks[location] = topic

            messagedata = BlockLocationMessage(
                block_id=topic, location=location, status=status
            )
            message_obj = MessageWrapper(topic=topic, message=messagedata)
            p = pickle.dumps(message_obj, protocol=-1)
            z = zlib.compress(p)
            print(f"{topic} {z}")

            socket.send_multipart([topic, z])
            time.sleep(3)
        except IndexError:
            print("Problem with block, check Arduino input and try again")
        except ValueError:
            print("Problem with block, check Arduino input and try again")
