from logzero import setup_default_logger, logfile, LogFormatter
import logging
import pyfiglet
from pkg_resources import resource_filename

DEBUG = True  # Use to control printing
communication = {
    "receive_messages_port": "tcp://0.0.0.0:5559",
}

LOGLEVEL = logging.INFO

LOGFILE = resource_filename("components", "simulator/logs/simulator_logfile.log")

banner = pyfiglet.figlet_format("Swarm Construction: Simulator")
print("\n")
print("*" * int(len(banner) * 0.15))
print(banner)
print("\n")
print("*" * int(len(banner) * 0.15))

print(LOGFILE)
with open(LOGFILE, "a") as file:
    file.write("\n")
    file.write("*" * int(len(banner) * 0.15))
    file.write("\n")
    file.write(banner)
    file.write("\nAuthors: Caleb Wagner and Hannan Liang\n")
    file.write("*" * int(len(banner) * 0.15))

log_format = "%(color)s[SIMULATOR: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d]%(end_color)s %(message)s"
log_file_format = (
    "[SIMULATOR: %(levelname)1.1s %(asctime)s %(module)s:%(lineno)d] %(message)s"
)
formatter = LogFormatter(fmt=log_format)
file_formatter = LogFormatter(fmt=log_file_format)

LOGGING = setup_default_logger(level=LOGLEVEL, formatter=formatter)
logfile(LOGFILE, maxBytes=1000000, backupCount=3, formatter=file_formatter)
