from uuid import uuid1
from pusher import Pusher

DEBUG = True  # Use to control printing
SIMULATE = True  # Use to control whether robot sends updates to simulator
communication = {
    "heartbeat_connection_in": "tcp://127.0.0.1:5555",
    "heartbeat_connection_out": "tcp://127.0.0.1:5556",
    "receive_messages_port": "tcp://127.0.0.1:5557",
    "send_messages_port": "tcp://127.0.0.1:5558",
    "simulator_send_messages_port": "tcp://127.0.0.1:5559",
}

TESTING = True  # Use to configure if need unique id or predefining id
ROBOT_ID = "ROBOT_1"
RECORD_METRICS = True

EXPERIMENT_NAME = "empire_state_building_2_robot"

if RECORD_METRICS:
    RECORD_BEHAVIOR_TIME_FILE = f"../results/behavior_time_{ROBOT_ID}_{EXPERIMENT_NAME}.csv"
else:
    RECORD_BEHAVIOR_TIME_FILE = None

pusher = Pusher(
    app_id="882780",
    key="b19a4591cdd9ad1d70f7",
    secret="eb1cdfa23df02681662e",
    cluster="us2",
    ssl=True,
)