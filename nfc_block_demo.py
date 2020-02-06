from components.robot.communication import *
from components.simulator.communication.messages import *
import zlib
import pickle
import serial
from uuid import uuid4

port = 'COM7' #change to serial port connected to homeblock arduino
ser = serial.Serial(port, 115200, timeout=None)

# ser.close() 
# ser.open() //use if port won't open

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://127.0.0.1:5559")

start = False
blocks = {}

while True:

    # TODONE: TREVOR, set the topic and location of blocks here

    line = str(ser.readline())[2:-5]
    if (line == "start"):
        start = True 
    elif (start and line != "" and line != "TIMEOUT!"):
        line = line.split(',')
        x = line[0]
        y = line[1]
        z = line[2]
        print(x, y, z)
        
        topic = b"BLOCK_" + str(uuid4()).encode()
        location = (float(x), float(y), float(z))

        messagedata = BlockLocationMessage(block_id=topic, location=location)
        message_obj = MessageWrapper(topic=topic, message=messagedata)
        p = pickle.dumps(message_obj, protocol=-1)
        z = zlib.compress(p)
        print(f"{topic} {z}")

        socket.send_multipart([topic, z])
        time.sleep(3)
