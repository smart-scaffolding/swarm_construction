import threading
import time
from collections import namedtuple

from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS

from components.robot.test.move_robot_new_test import robot_trajectory_serial_demo

Point = namedtuple("Point", "x y z direction holding_block")

##############################################################################
# Serial
##############################################################################
"""
Change these variables for controlling serial parameters:

SERIAL: Used to enable or disable sending commands to physical robots. Set to false if only want to simulate, 
        set to true if want to send to robot 

PORT: A string representing port the usb is plugged into (check Arduino IDE for your port under Tools)

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

TIMEOUT: The rate each set of angles is sent at (ie new angles sent every 1s). Measured in seconds.
        CAUTION: Be careful setting this number less than 0.2s. It is safe to try, but it may freak out

NUM_VIA_POINTS: The number of via points between each waypoint. Note that the total number of angles will be three 
                times the number of via points, as there are three waypoints for every motion. 
        CAUTION: Do not set this number less than 2

"""

TIMEOUT = 0.02  # seconds 0.03
NUM_VIA_POINTS = 50  # 25

##############################################################################
# Path Selection
##############################################################################
path = [(3, 0, 0, "top"), (1, 0, 0, "top")]

##############################################################################
# Gripper Control
##############################################################################
"""
Used to select whether the grippers are actually used.


USE_GRIPPERS: True if grippers are to be used, False if they are not to be used
    CAUTION: The grippers may not detach completely, and the robot may try and move while the gripper(s) are still 
             engaged. Make sure that you are watching for this and unplug the robot before the motors stall     
"""

USE_GRIPPERS = False


def read_angles_from_robot(read_serial):
    print("Ready to read angles\n")
    while True:
        if read_serial.in_waiting > 0:
            try:
                line = read_serial.readline()
                print(f"[ROBOT]: {line.decode()}")
            except Exception as e:
                print(e)
                continue


def get_command_line_input(arguments, index):
    output = None
    try:
        output = arguments[index]
    except ValueError:
        pass

    return output


def get_path(case):
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
            Point(3, 0, 1, "top", None),
            Point(2, 0, 0, "top", None),
            Point(4, 0, 2, "top", None),
            Point(3, 0, 1, "top", None),
            Point(5, 0, 3, "top", None),
            Point(4, 0, 2, "top", None),
        ]
    else:
        return [Point(3, 0, 0, "top", None)]


def convert_js_bool(output):
    return True if output == "true" else False


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        output = get_command_line_input(sys.argv, 1)
        SERIAL = SERIAL if output is None else convert_js_bool(str(output))

        output = get_command_line_input(sys.argv, 2)
        PORT = PORT if output is None else str(output)

        output = get_command_line_input(sys.argv, 3)
        BAUD = BAUD if output is None else int(output)

        output = get_command_line_input(sys.argv, 4)
        TIMEOUT = TIMEOUT if output is None else float(output)

        output = get_command_line_input(sys.argv, 5)
        NUM_VIA_POINTS = NUM_VIA_POINTS if output is None else int(output)

        output = get_command_line_input(sys.argv, 6)
        USE_GRIPPERS = USE_GRIPPERS if output is None else convert_js_bool(str(output))

        output = get_command_line_input(sys.argv, 7)
        path = path if output is None else get_path(str(output))

    print(f"Serial: {SERIAL}")
    print(f"Port: {PORT}")
    print(f"BAUD: {BAUD}")
    print(f"Delay: {TIMEOUT}")
    print(f"Via Points: {NUM_VIA_POINTS}")
    print(f"Grippers: {USE_GRIPPERS}")
    print(f"Trajectory: {path}")

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
