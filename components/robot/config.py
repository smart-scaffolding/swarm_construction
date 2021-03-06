import logging

import pyfiglet
from logzero import setup_default_logger, LogFormatter
from pusher import Pusher

from swarm_c_library.blueprint_factory import BluePrintFactory

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


#
LOGLEVEL = logging.DEBUG
# LOGFILE = "./logs/robot_logfile.log"
#
#
banner = pyfiglet.figlet_format("Swarm Construction: Robot")
print("\n")
print("*" * int(len(banner) * 0.15))
print(banner)
print("\n")
print("*" * int(len(banner) * 0.15))
#
# with open(LOGFILE, "a") as file:
#     file.write("\n")
#     file.write("*" * int(len(banner)*0.15))
#     file.write("\n")
#     file.write(banner)
#     file.write("\nAuthors: Caleb Wagner and Hannan Liang\n")
#     file.write("*" * int(len(banner)*0.15))
#
log_format = f"%(color)s[{ROBOT_ID}: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d]%(end_color)s %(message)s"
# log_file_format = f'[{ROBOT_ID}: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d] %(message)s'
#
formatter = LogFormatter(fmt=log_format)
# file_formatter = LogFormatter(fmt=log_file_format)
LOGGING = setup_default_logger(level=LOGLEVEL, formatter=formatter)
# logfile(LOGFILE, maxBytes=1000000, backupCount=3, formatter=file_formatter)

pusher = Pusher(
    app_id="882780",
    key="b19a4591cdd9ad1d70f7",
    secret="eb1cdfa23df02681662e",
    cluster="us2",
    ssl=True,
)


# blueprint1 = np.array([[[1] * 1] * 10] * 10)
#
# blueprint2 = np.array([[[0] * 1] * 10] * 10)
#
# blueprint3 = np.array([[[0] * 1] * 10] * 10)
#
# blueprint4 = np.array([[[0] * 1] * 10] * 10)
#
# blueprints = [blueprint1, blueprint2, blueprint3, blueprint4]
#
# BLUEPRINT = np.dstack(blueprints)

BLUEPRINT = BluePrintFactory().get_blueprint("Plane_20x20x1").data
