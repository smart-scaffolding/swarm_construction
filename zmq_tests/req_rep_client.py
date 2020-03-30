#
#   Request-reply client in Python
#   Connects REQ socket to tcp://localhost:5559
#   Sends "Hello" to server, expects "World" back
#
import zmq

# context = zmq.Context()
# socket = context.socket(zmq.REP)
# socket.connect("tcp://127.0.0.1:5560")


ports = ["tcp://127.0.0.1:5561"]

context = zmq.Context()
# print("Connecting to machine...")

sockets = []
for index in range(len(ports)):
    context = zmq.Context()
    print("Connecting to machine...")
    socket = context.socket(zmq.REP)
    # socket.setsockopt(zmq.LINGER, 0)
    # socket.bind("%s" % ports[index])
    socket.connect(ports[index])
    print("Successfully connected to machine %s" % ports[index])
    sockets.append(socket)


while True:
    for index in range(len(ports)):
        message = sockets[index].recv_string()
        print("Sending position to structure: ", message)
        sockets[index].send_string(f"Position: {ports[index]}")
        # time.sleep(3)
