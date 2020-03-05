from components.robot.test.move_robot_new_test import robot_trajectory_serial_demo
import numpy as np
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
import threading
import time


##############################################################################
# Serial
##############################################################################
'''
Change these variables for controlling serial parameters:

SERIAL: Used to enable or disable sending commands to physical robots. Set to false if only want to simulate, 
        set to true if want to send to robot 

PORT: A string representing port the usb is plugged into (check Arduino IDE for your port under Tools)

BAUD: The baud rate
'''

SERIAL = True
PORT = '/dev/cu.usbmodem14201'
# PORT = '/dev/cu.usbserial-DN02P9MR'

BAUD = 115200



##############################################################################
# Trajectory Parameters
##############################################################################
'''
Change these parameters for adjusting trajectory parameters

TIMEOUT: The rate each set of angles is sent at (ie new angles sent every 1s). Measured in seconds.
        CAUTION: Be careful setting this number less than 0.2s. It is safe to try, but it may freak out

NUM_VIA_POINTS: The number of via points between each waypoint. Note that the total number of angles will be three 
                times the number of via points, as there are three waypoints for every motion. 
        CAUTION: Do not set this number less than 2
                 
'''

TIMEOUT = 0.02          # seconds 0.03
NUM_VIA_POINTS = 25     # 25


##############################################################################
# Path Selection
##############################################################################
'''
Use to select which path (trajectory) the robot will perform. Make sure that you comment out the paths that you do 
not wish to run and uncomment the single path you do wish to run.
'''

# D link moves forward one step
# path = [(3, 0, 0, 'top')]

# D link moves forward one step and will stop at block height (use to reach block)
# path = [(3, 0, 1, 'top')]

# Inch from start to end
## NOTE: Grippers must either be enabled or disengaged for this to work
# path = [(3, 0, 0, "top"), (1, 0, 0, "top"), (4, 0, 0, "top"), (2, 0, 0, "top"), (5, 0, 0, "top"), (3, 0, 0, "top")]

# path = [(3, 0, 0, 'top'), (1, 0, 0, "top")]

# Move block forward
## NOTE: Grippers must either be enabled or disengaged for this to work
## NOTE: Block must be placed underneath for robot to step on
path=[(3, 0, 1, "top"), (2, 0, 0, "top")]


# Move block forward
## NOTE: Grippers must either be enabled or disengaged for this to work
## NOTE: Block must be placed underneath for robot to step on
# path=[(3, 0, 1, "top"), (2, 0, 0, "top"), (4, 0, 2, "top")]
# path = [(3, 1, 1, "top"), (2, 1, 0, "top"), (4, 1, 2, "top"), (3, 1, 1, "top"), (5, 1, 3, "top"), (4, 1, 2,
#                                                                                                        "top")]
# path = [(3, 0, 1, "top"), (2, 0, 0, "top"), (4, 0, 1, "top"), (3, 0, 0, "top"), (5, 0, 1, "top"), (4, 0, 1,
#                                                                                                        "top"), (6, 0,
#                                                                                                                 2,
#                                                                                                                 "top"), (5, 0, 1, "top"), (7, 0, 3, "top"), (6, 0, 2, "top")]
##############################################################################
# Gripper Control
##############################################################################
'''
Used to select whether the grippers are actually used.


USE_GRIPPERS: True if grippers are to be used, False if they are not to be used
    CAUTION: The grippers may not detach completely, and the robot may try and move while the gripper(s) are still 
             engaged. Make sure that you are watching for this and unplug the robot before the motors stall     
'''

USE_GRIPPERS = True


robot_serial = Serial(port=PORT, baudrate=BAUD, parity=PARITY_NONE,
                                    stopbits=STOPBITS_ONE, bytesize=EIGHTBITS, timeout=3.0)


def read_angles_from_robot(read_serial):
    print("Ready to read angles\n")
    while True:
        if read_serial.in_waiting > 0:
            try:
                line = read_serial.readline()
                # print(f"[ROBOT]: {line.decode()}")
            except Exception as e:
                print(e)
                continue

reading = threading.Thread(target=read_angles_from_robot, args=(robot_serial,))
reading.start()

time.sleep(0.5)
robot_trajectory_serial_demo(num_steps=NUM_VIA_POINTS, baud=BAUD, serial=SERIAL, timeout=TIMEOUT, port=PORT,
                             path=path, use_grippers=USE_GRIPPERS)
