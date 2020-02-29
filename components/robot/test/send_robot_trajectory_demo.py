from components.robot.test.move_robot_new_test import robot_trajectory_serial_demo
import numpy as np


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

SERIAL = False
PORT = '/dev/cu.usbmodem14201'
BAUD = 9600



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

TIMEOUT = 3.0           # seconds
NUM_VIA_POINTS = 50


##############################################################################
# Path Selection
##############################################################################
'''
Use to select which path (trajectory) the robot will perform. Make sure that you comment out the paths that you do 
not wish to run and uncomment the single path you do wish to run.
'''

# D link moves forward one step
path = [(3, 0, 0, 'top')]

# D link moves forward one step and will stop at block height (use to reach block)
# path = [(3, 0, 1, 'top')]



# Inch from start to end
## NOTE: Grippers must either be enabled or disengaged for this to work
# path = [(3, 0, 0, "top"), (1, 0, 0, "top"), (4, 0, 0, "top"), (2, 0, 0, "top"), (5, 0, 0, "top"), (3, 0, 0, "top")]



##############################################################################
# Gripper Control
##############################################################################
'''
Used to select whether the grippers are actually used.


USE_GRIPPERS: True if grippers are to be used, False if they are not to be used
    CAUTION: The grippers may not detach completely, and the robot may try and move while the gripper(s) are still 
             engaged. Make sure that you are watching for this and unplug the robot before the motors stall     
'''

USE_GRIPPERS = False


robot_trajectory_serial_demo(num_steps=NUM_VIA_POINTS, baud=BAUD, serial=SERIAL, timeout=TIMEOUT, port=PORT,
                             path=path, use_grippers=USE_GRIPPERS)
