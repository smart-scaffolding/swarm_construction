from collections import namedtuple
from components.robot.main import RobotMain
from components.structure.behaviors.building.common_building import Block
from components.robot.original.move_robot_new import robot_trajectory_serial_demo
from components.robot.communication.messages import *
import time
Point = namedtuple("Point", "x y z direction holding_block")

##############################################################################
# Serial
##############################################################################
"""
Change these variables for controlling serial parameters:

SERIAL: Used to enable or disable sending commands to physical robots. Set 
to false if only want to simulate, 
        set to true if want to send to robot 

PORT: A string representing port the usb is plugged into (check Arduino IDE 
for your port under Tools)

BAUD: The baud rate
"""

SERIAL = False
PORT = "/dev/cu.usbmodem14201"
# PORT = '/dev/cu.usbserial-DN02P9MR'

if not SERIAL:
    PORT = None
BAUD = 115200

##############################################################################
# Trajectory Parameters
##############################################################################
"""
Change these parameters for adjusting trajectory parameters

TIMEOUT: The rate each set of angles is sent at (ie new angles sent every 
1s). Measured in seconds.
        CAUTION: Be careful setting this number less than 0.2s. It is safe 
        to try, but it may freak out

NUM_VIA_POINTS: The number of via points between each waypoint. Note that 
the total number of angles will be three 
                times the number of via points, as there are three waypoints 
                for every motion. 
        CAUTION: Do not set this number less than 2                 
"""

TIMEOUT = 0.02  # seconds 0.03
NUM_VIA_POINTS = 5  # 25

##############################################################################
# Path Selection
##############################################################################
"""
Use to select which path (trajectory) the robot will perform. Make sure that 
you comment out the paths that you do 
not wish to run and uncomment the single path you do wish to run.
"""
block_id = "BLOCK_1"
# D link moves forward one step
path = [
    Point(3, 2, 5, "top", block_id),
    Point(3, 1, 4, "top", None),

    Point(3, 4, 5, "top", block_id),
    Point(3, 3, 4, "top", None),
    
    Point(3, 6, 4, "top", block_id),
    Point(3, 5, 4, "top", None),
    
    Point(4, 7, 4, "top", block_id),
    Point(4, 5, 3, "top", None),
    
    Point(4, 8, 4, "top", block_id),
    Point(4, 6, 3, "top", None),
    
    Point(4, 9, 4, "top", block_id),
    Point(4, 7, 3, "top", None),
    
    Point(3, 10, 4, "top", block_id),
    Point(3, 8, 3, "top", None),

    Point(3, 11, 4, "top", block_id),
    Point(3, 9, 3, "top", None),

    # Point(3, 8, 3, "top", None),

    # Point(4, 12, 3, "top", None),
    # Point(3, 9, 3, "top", None),
]

a = 11
d = 9
for i in range(9):
    a += 1
    d += 1
    path.append(Point(3, a, 4, "top", block_id))
    path.append(Point(3, d, 3, "top", None))
a+=1
d+=1

path.append(Point(3, a, 3, "top", block_id))
path.append(Point(3, d, 3, "top", None))
path.append(Point(3, a, 2, "top", block_id))
path.append(Point(3, d, 3, "top", None))

# a -= 1
path.append(Point(3, a-1, 3, "top", None))


for i in range(13):
    a-= 1
    d -= 1
    path.append(Point(3, d, 3, "top", None))
    path.append(Point(3, a-1, 3, "top", None))
# path.append(Point(3, d, 3, "top", None))

# path = [
#     Point(2, 2, 0, "top", None),
#     Point(2, 4, 0, "top", None),
#     Point(2, 6, 0, "top", None),
#     Point(2, 8, 0, "top", None),
# ]

# path = [
#     Point(2, 2, 0, "top", None),
#     Point(4, 4, 0, "top", None),
#     Point(6, 6, 0, "top", None),
#     Point(8, 8, 0, "top", None),
# ]

# block_id = "BLOCK_1"
# block_id_2 = "BLOCK_2"
# block_id_3 = "BLOCK_3"
# path = [
#     Point(0, 0, 1, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(4, 0, 1, "top", block_id),
#     Point(3, 0, 0, "top", None),
#     Point(5, 0, 1, "top", block_id),
#     Point(4, 0, 0, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(3, 0, 0, "top", None),
#     Point(0, 0, 1, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(4, 0, 1, "top", block_id_2),
#     Point(3, 0, 0, "top", None),
#     Point(5, 0, 2, "top", block_id_2),
#     Point(4, 0, 0, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(3, 0, 0, "top", None),
#     Point(0, 0, 1, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(4, 0, 1, "top", block_id_3),
#     Point(3, 0, 0, "top", None),
#     Point(5, 0, 2, "top", None),
#     Point(4, 0, 1, "top", None),
#     Point(5, 1, 3, "top", None),
#     Point(5, 0, 2, "top", None),
#     Point(4, 2, 3, "top", None),
#     Point(5, 1, 3, "top", None),
#     Point(3, 1, 1, "top", None),
#     Point(4, 1, 2, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(0, 0, 0, "top", None),
# ]
destinations = [
        ("BLOCK_1", (0, 0, 1)),
        ("BLOCK_2", (0, 0, 1)),
        ("BLOCK_3", (0, 0, 1))

    ]

blocks = []
for block_id, destination in destinations:
    block = Block(final_destination=(8, 6, 1))
    block.set_next_location(destination)
    block.location = destination
    block.id = block_id
    blocks.append(block)
blocks.reverse()

# print(blocks)
# robot = RobotMain()
# robot.initialize_communications()
# time.sleep(3)
# for block in blocks:
#     robot.simulator_communicator.send_communication(
#         topic=block.id.encode(),
#         message=BlockLocationMessage(
#             block_id=block.id,
#             location=(
#                 block.location[0],
#                 block.location[1] + 0.5,
#                 block.location[2] + 0.5,
#             ),
#         ),
#     )
#     print(f"Sending block with id {block.id}")

# D link moves forward one step and will stop at block height (use to reach
# block)
# path = [(3, 0, 1, 'top')]

# Inch from start to end
# NOTE: Grippers must either be enabled or disengaged for this to work
# path = [
#     (3, 0, 0, "top"),
#     (1, 0, 0, "top"),
#     (4, 0, 0, "top"),
#     (2, 0, 0, "top"),
#     (5, 0, 0, "top"),
#     (3, 0, 0, "top"),
# ]

# path = [Point(3, 0, 0, "top", None), Point(1, 0, 0, "top", None)]

# Move block forward
# NOTE: Grippers must either be enabled or disengaged for this to work
# NOTE: Block must be placed underneath for robot to step on
# path = [(3, 0, 1, "top"), (2, 0, 0, "top")]


# Move block forward
# NOTE: Grippers must either be enabled or disengaged for this to work
# NOTE: Block must be placed underneath for robot to step on

# block_id = "1"
# block_id_2 = "2"
# path = [
#     Point(3, 0, 1, "top", block_id),
#     Point(2, 0, 0, "top", None),
#     Point(4, 0, 1, "top", block_id),
#     Point(3, 0, 0, "top", None),
#     Point(5, 0, 1, "top", block_id),
#     Point(4, 0, 0, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(3, 0, 0, "top", None),
#     Point(0, 0, 1, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(4, 0, 1, "top", block_id_2),
#     Point(3, 0, 0, "top", None),
#     Point(5, 0, 2, "top", block_id_2),
#     Point(4, 0, 0, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(3, 0, 0, "top", None),
#     Point(0, 0, 1, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(4, 0, 1, "top", block_id_2),
#     Point(3, 0, 0, "top", None),
#     Point(2, 0, 0, "top", None),
#     Point(3, 0, 0, "top", None),
#     Point(0, 0, 0, "top", None),
# ]
#
#
# path = [
#     (3, 1, 1, "top"),
#     (2, 1, 0, "top"),
#     (4, 1, 2, "top"),
#     (3, 1, 1, "top"),
#     (5, 1, 3, "top"),
#     (4, 1, 2, "top"),
# ]
# path = [
#     (3, 0, 1, "top"),
#     (2, 0, 0, "top"),
#     (4, 0, 1, "top"),
#     (3, 0, 0, "top"),
#     (5, 0, 1, "top"),
#     (4, 0, 1, "top"),
#     (6, 0, 2, "top"),
#     (5, 0, 1, "top"),
#     (7, 0, 3, "top"),
#     (6, 0, 2, "top"),
# ]
#
# path = [
#     (3, 1, 1, "top"),
#     (2, 1, 0, "top"),
#     (4, 1, 2, "top"),
#     (3, 1, 1, "top"),
#     (5, 1, 3, "top"),
#     (4, 1, 2, "top"),
# ]
##############################################################################
# Gripper Control
##############################################################################
"""
Used to select whether the grippers are actually used.


USE_GRIPPERS: True if grippers are to be used, False if they are not to be used
    CAUTION: The grippers may not detach completely, and the robot may try 
    and move while the gripper(s) are still 
             engaged. Make sure that you are watching for this and unplug 
             the robot before the motors stall     
"""

USE_GRIPPERS = False


def read_angles_from_robot(read_serial):
    """

    :param read_serial:
    """
    print("Ready to read angles\n")
    while True:
        if read_serial.in_waiting > 0:
            try:
                line = read_serial.readline()
                print(f"[ROBOT]: {line.decode()}")
            except Exception as e:
                print(e)
                continue


# are the datas shared through stigmergy guaranteed to be global across all
# robots (ie do all robots have access to
# the exact same information?

# are there any race conditions, where robots might be acting on old
# information?


# how is the distance between robots calculated (ie ultrasonic, lidar,
# rangefinders)

# The example shown in the slides only showed the distance between robots as
# a scalar, is it in fact a vector?

# How do robots know how to update their own distances if they receive data
# from multiple other robots? Is there some
# kind of filtering that is being used to determine which robots to listen
# to regarding distance?


# Based on the way buzz works with timesteps, are there chances that cause
# these timesteps to be slowed down,
# ie if a certain operation takes longer than a single timestep? Does this
# mean timesteps will not be uniform


def get_command_line_input(arguments, index):
    """

    :param arguments:
    :param index:
    :return:
    """
    output = None
    try:
        output = arguments[index]
    except ValueError:
        pass

    return output


def get_path(case):
    """

    :param case:
    :return:
    """
    if case == "single_step":
        return [Point(3, 0, 0, "top", None)]
    if case == "single_step_block":
        return [Point(3, 0, 1, "top", None)]
    if case == "two_step":
        return [Point(3, 0, 0, "top", None), Point(1, 0, 0, "top", None)]
    if case == "two_step_onto_block":
        return [Point(3, 0, 1, "top", None), Point(1, 0, 0, "top", None)]
    if case == "full_playground_inch":
        return [
            Point(3, 0, 0, "top", None),
            Point(1, 0, 0, "top", None),
            Point(4, 0, 0, "top", None),
            Point(2, 0, 0, "top", None),
            Point(5, 0, 0, "top", None),
            Point(3, 0, 0, "top", None),
        ]
    if case == "stairs":
        return [
            Point(3, 1, 1, "top", None),
            Point(2, 1, 0, "top", None),
            Point(4, 1, 2, "top", None),
            Point(3, 1, 1, "top", None),
            Point(5, 1, 3, "top", None),
            Point(4, 1, 2, "top", None),
        ]
    else:
        return [Point(3, 0, 0, "top", None)]


def convert_js_bool(output):
    """

    :param output:
    :return:
    """
    return True if output == "true" else False


if __name__ == "__main__":
    import sys

    # if len(sys.argv) > 1:
    #     output = get_command_line_input(sys.argv, 1)
    #     SERIAL = SERIAL if output is None else convert_js_bool(str(output))

    #     output = get_command_line_input(sys.argv, 2)
    #     PORT = PORT if output is None else str(output)

    #     output = get_command_line_input(sys.argv, 3)
    #     BAUD = BAUD if output is None else int(output)

    #     output = get_command_line_input(sys.argv, 4)
    #     TIMEOUT = TIMEOUT if output is None else float(output)

    #     output = get_command_line_input(sys.argv, 5)
    #     NUM_VIA_POINTS = NUM_VIA_POINTS if output is None else int(output)

    #     output = get_command_line_input(sys.argv, 6)
    #     USE_GRIPPERS = USE_GRIPPERS if output is None else convert_js_bool(str(output))

    #     output = get_command_line_input(sys.argv, 7)
    #     path = path if output is None else get_path(str(output))

    print(f"Serial: {SERIAL}")
    print(f"Port: {PORT}")
    print(f"BAUD: {BAUD}")
    print(f"Delay: {TIMEOUT}")
    print(f"Via Points: {NUM_VIA_POINTS}")
    print(f"Grippers: {USE_GRIPPERS}")
    print(f"Trajectory: {path}")

    # if SERIAL:
    #     robot_serial = Serial(port=PORT, baudrate=BAUD, parity=PARITY_NONE,
    #                           stopbits=STOPBITS_ONE, bytesize=EIGHTBITS,
    #                           timeout=3.0)
    #     reading = threading.Thread(target=read_angles_from_robot,
    #     args=(robot_serial,))
    #     reading.start()
    #
    # time.sleep(0.5)
    robot_trajectory_serial_demo(
        num_steps=NUM_VIA_POINTS,
        baud=BAUD,
        serial=SERIAL,
        timeout=TIMEOUT,
        port=PORT,
        path=path,
        use_grippers=USE_GRIPPERS,
    )
