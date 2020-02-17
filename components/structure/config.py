from uuid import uuid1

DEBUG = True  # Use to control printing
SIMULATE = True # Use to control whether robot sends updates to simulator
communication = {
    "heartbeat_connection_in": "tcp://0.0.0.0:5555",
    "heartbeat_connection_out": "tcp://0.0.0.0:5556",
    "receive_messages_port": "tcp://0.0.0.0:5558",
    "send_messages_port": "tcp://0.0.0.0:5557",
    "simulator_send_messages_port": "tcp://0.0.0.0:5559",
}

TESTING = True  # Use to configure if need unique id or predefining id
ROBOT_ID = "ROBOT_1"