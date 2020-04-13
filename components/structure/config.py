from uuid import uuid1

DEBUG = True  # Use to control printing
SIMULATE = True  # Use to control whether robot sends updates to simulator
communication = {
    "heartbeat_connection_in": "tcp://127.0.0.1:5555",
    "heartbeat_connection_out": "tcp://127.0.0.1:5556",
    "receive_messages_port": "tcp://127.0.0.1:5558",
    "send_messages_port": "tcp://127.0.0.1:5557",
    "simulator_send_messages_port": "tcp://127.0.0.1:5559",
}

TESTING = True  # Use to configure if need unique id or predefining id
ROBOT_ID = "ROBOT_1"

# Use for creation of results files to label properly
EXPERIMENT_NAME = "simulator_results_empire_state_building_2_robots.csv"
PATH_TO_RESULTS = "results/empire/"  # Path to where results will be stored
BLUEPRINT = "Temple"
SIMULATOR_BLUEPRINT = "Plane_20x20x1"
