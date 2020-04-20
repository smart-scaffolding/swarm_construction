from uuid import uuid1
from components.structure.behaviors.building.common_building import (
    Robot,
)

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

# Use for creation of results files to label properly
EXPERIMENT_NAME = "simulator_results_taj_mahal_2_robots.csv"
PATH_TO_RESULTS = "results/tajmahal/"  # Path to where results will be stored
BLUEPRINT = "StarTrek"
SIMULATOR_BLUEPRINT = "Plane_10x10x1"

ROBOTS = [
        Robot(id=b"ROBOT_1", pos=(1.5, 1.5), claimed_division=1),
        Robot(id=b"ROBOT_2", pos=(4.5, 1.5), claimed_division=2),
        # Robot(id=b'ROBOT_3', pos=(4.5, 4.5), claimed_division=3),
        # Robot(id=b'ROBOT_4', pos=(7.5, 1.5), claimed_division=4),
        ]

DIVISION_SIZE = 5