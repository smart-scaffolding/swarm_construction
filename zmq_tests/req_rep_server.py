#
#   Request-reply service in Python
#   Connects REP socket to tcp://localhost:5560
#   Expects "Hello" from client, replies with "World"
#

import time

import zmq

#  Prepare our context and sockets
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.bind("tcp://127.0.0.1:5561")

#  Do 10 requests, waiting each time for a response
for request in range(1, 11):
    print(f"Requesting position: [{request}/10]")
    socket.send_string(f"Please tell me your position: {request}")
    message = socket.recv_string()
    print("Received position %s [%s]" % (request, message))
    time.sleep(7)
