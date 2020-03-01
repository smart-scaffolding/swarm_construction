from logzero import setup_default_logger, logfile, LogFormatter
import logging

DEBUG = True  # Use to control printing
communication = {
    "receive_messages_port": "tcp://0.0.0.0:5559",
}

LOGLEVEL = logging.DEBUG
LOGFILE = "./logs/simulator_logfile.log"

log_format = '%(color)s[SIMULATOR: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d]%(end_color)s %(message)s'
log_file_format = '[SIMULATOR: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d] %(message)s'
formatter = LogFormatter(fmt=log_format)
file_formatter = LogFormatter(fmt=log_file_format)

LOGGING = setup_default_logger(level=LOGLEVEL, formatter=formatter)
logfile(LOGFILE, maxBytes=1000000, backupCount=3, formatter=file_formatter)