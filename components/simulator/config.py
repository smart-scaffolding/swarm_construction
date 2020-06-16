import logging

import pyfiglet
from logzero import LogFormatter, logfile, setup_default_logger
from pkg_resources import resource_filename

from components.simulator.model.graphics import VtkPipeline
from swarm_c_library.blueprint_factory import BluePrintFactory

communication = {
    "receive_messages_port": "tcp://0.0.0.0:5559",
}

# NOTE: This blueprint is meant to be the existing structure that already exists, not the one the robot will create
BLUEPRINT = BluePrintFactory().get_blueprint("Playground").data
bx, by, bz = BLUEPRINT.shape
COLORS = [[[VtkPipeline.vtk_named_colors(["DarkGreen"])] * bz] * by] * bx

# Frequency at which number of timesteps will be displayed (DEFAULT: 100)
# NOTE: Log level must be set to DEBUG to view
PRINT_TIMER_FREQUENCY = 100

# If on a Mac (only Mac supported), this will create an alarm that will ring once the simulation has finished
NOTIFY_WHEN_SIMULATION_FINISHED = True
LOGLEVEL = logging.INFO


"""
BELOW IS INFORMATION RELATED TO LOGGING ONLY

"""
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
