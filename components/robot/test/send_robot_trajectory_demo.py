import threading
import time
from collections import namedtuple

from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS

from components.robot.test.move_robot_new import robot_trajectory_serial_demo

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

TIMEOUT = 0.02
NUM_VIA_POINTS = 20

##############################################################################
# Path Selection
##############################################################################
"""
Use to select which path (trajectory) the robot will perform. Make sure that 
you comment out the paths that you do 
not wish to run and uncomment the single path you do wish to run.
"""


def get_path_from_input(case):
    """
    Returns the desired path
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


"""
OPTIONS:

single_step          -> Robot will take a single step
single_step_block    -> Robot will step onto of block
two_step             -> Robot will move both back and front end effectors, NEED GRIPPERS ENABLED
two_step_onto_block  -> Robot will step onto block and then move back ee, NEED BLOCK AND GRIPPERS
full_playground_inch -> Will inch length of playground, NEED GRIPPERS ENABLED
stairs               -> Robot will climb stairs, ensure robot is placed by stairs properly, NEED GRIPPERS ENABLED

"""
PATH_SELECTION = "single_step"

path = get_path_from_input(PATH_SELECTION)


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
    Reads the angles from the robot over serial
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


if __name__ == "__main__":

    if SERIAL:
        robot_serial = Serial(
            port=PORT,
            baudrate=BAUD,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            bytesize=EIGHTBITS,
            timeout=3.0,
        )
        reading = threading.Thread(target=read_angles_from_robot, args=(robot_serial,))
        reading.start()

    time.sleep(0.5)
    robot_trajectory_serial_demo(
        num_steps=NUM_VIA_POINTS,
        baud=BAUD,
        serial=SERIAL,
        timeout=TIMEOUT,
        port=PORT,
        path=path,
        use_grippers=USE_GRIPPERS,
    )
