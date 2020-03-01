from uuid import uuid1
from logzero import setup_default_logger, logfile, LogFormatter
import logging


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

LOGLEVEL = logging.DEBUG
LOGFILE = "./logs/structure_logfile.log"
log_format = '%(color)s[STRUCTURE: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d]%(end_color)s %(message)s'
log_file_format = '[STRUCTURE: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d] %(message)s'

formatter = LogFormatter(fmt=log_format)
file_formatter = LogFormatter(fmt=log_file_format)

LOGGING = setup_default_logger(level=LOGLEVEL, formatter=formatter)
logfile(LOGFILE, maxBytes=1000000, backupCount=3, formatter=file_formatter)
