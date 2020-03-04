import components.robot.test.model as model
import numpy as np
import math
import time
from components.robot.test.common import create_point_from_homogeneous_transform, flip_base, round_end_effector_position
from components.robot.test.quintic_trajectory_planner import *
from components.robot.communication.messages import *
from components.robot.pathplanning.searches.face_star import *
from components.robot.test.transforms import *
from components.robot.test.minimum_jerk_trajectory_planner import *
# Comms
import zmq
import zlib
import pickle
import components.robot.config as config
from random import randint, choice
from datetime import datetime


accuracy = 1e-7
threshold = 1
use_face_star = False
SIMULATE = True

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://127.0.0.1:5559")
TOPIC = b"ROBOT_1"

JOINT_ANGLE_PKT_SIZE = 8

robot_ee_starting_point = (2.5, 0.5, 1)

blueprint = np.array([
    [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
    [[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[1, 0, 0], [1, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[1, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[1, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[1, 1, 1], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
])

# playground
blueprint = np.array([
    [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
    [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
    [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
])

base = np.matrix([[1, 0, 0, 0.5],
                  [0, 1, 0, 0.5],
                  [0, 0, 1, 1.],
                  [0, 0, 0, 1]])


class AnimationUpdate:
    def __init__(self, robot, robot_base, index, direction, trajectory, path, placedObstacle=False, obstacle=None):
        self.robot = robot
        self.robot_base = robot_base
        self.index = index
        self.direction = direction
        self.trajectory = trajectory
        self.path = path
        self.placedObstacle = placedObstacle
        self.obstacle = obstacle


def robot_trajectory_serial_demo(num_steps, serial, port, baud, timeout, path, blueprint=blueprint, base=base,
                                 velocity_offset=0, use_grippers=False):
    robot = model.Inchworm(base=base, blueprint=blueprint)


    ik_motion, path, directions, animation_update = follow_path(robot, num_steps, offset=1.20, path=path)

    robot = model.Inchworm(base=base, blueprint=blueprint, port=port, baud=baud)

    flip_angles = True
    first_serial_message = True
    for index, angle in enumerate(ik_motion):
        # print(f"Angle: {angle}")

        angle = angle.tolist()[0]
        # robot.map_angles_to_robot(angle)
        if index % (num_steps) == 0:
            print("\nFIRST POINT REACHED")

        if index % (num_steps * 2) == 0:
            print("\n SECOND STEP REACHED")
        if index % (num_steps * 3) == 0:
            print("\n THIRD STEP REACHED")
            flip_angles = True if flip_angles == False else False
            if serial and use_grippers:
                if first_serial_message:
                    if flip_angles:
                        robot.send_to_robot(angle, delay=timeout, open_gripper="21", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    else:
                        robot.send_to_robot(angle, delay=timeout, open_gripper="11", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)

                    first_serial_message = False
                    time.sleep(2)

                if flip_angles:
                    time.sleep(2)
                    robot.send_to_robot(angle, open_gripper="21", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    robot.send_to_robot(angle, delay=timeout, open_gripper="21", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    time.sleep(1)
                    robot.send_to_robot(angle, delay=timeout, open_gripper="12", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    time.sleep(45)
                else:
                    time.sleep(2)
                    robot.send_to_robot(angle, delay=timeout, open_gripper="11", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    robot.send_to_robot(angle, delay=timeout, open_gripper="11", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    time.sleep(1)
                    robot.send_to_robot(angle, delay=timeout, open_gripper="22", index=index, total_num_points=len(ik_motion), velocity_offset=velocity_offset)
                    time.sleep(45)
            print("\n\nIndex: {}  New Flipping Angle: {}".format(index, flip_angles))

        if flip_angles:
            temp = angle[1]
            angle[1] = 180 / 2 + angle[3]
            angle[3] = temp - 180 / 2

        if serial:
            print("Sending angle to robot")
            robot.send_to_robot(angle, index=index, total_num_points=len(ik_motion), velocity_offset=0,
                                delay=timeout)


def move_to_point(direction, point, robot, num_steps, baseID, previous_angles=None, accuracy=accuracy):

    if baseID == 'A':
        currentEEPos = robot.DEE_POSE[:3,3]
        basePos = robot.AEE_POSE[:3,3]
    else:
        currentEEPos = robot.AEE_POSE[:3,3]
        basePos = robot.DEE_POSE[:3,3]

    check_if_point_reachable(robot, basePos, point)

    setPoints, _, velPoints, _ = get_quintic_trajectory(points=np.array([currentEEPos,point]), set_points=num_steps)

    # setPoints, _, _, _ = get_minimum_jerk_trajectory(points=np.array([currentEEPos, point]), average_velocity=3.0,
    #                                                  frequency=50)

    forward_1 = []
    forward_2 = []
    forward_3 = []
    forward_4 = []
    base = robot.AEE_POSE

    for index, point in enumerate(setPoints):
        gamma = temp_direction_to_gamma_convertion(direction)
        ik_angles = robot.ikin(goalPos=point,gamma=gamma,phi=0,baseID=baseID,simHuh=True)
        ik_angles = map_angles_from_robot_to_simulation(ik_angles)

        # TODO: refactor the code
        # Temporary fix for base flipping
        if baseID == 'D':
            # ik_angles[0] = ik_angles[0] - 180
            base = robot.DEE_POSE

        forward_1.append(ik_angles[0])
        forward_2.append(ik_angles[1])
        forward_3.append(ik_angles[2])
        forward_4.append(ik_angles[3])
        # print(f'each ik_angle {ik_angles}')

        if previous_angles is None:
            previous_angles = [1] * robot.length

        angle_update = ik_angles
        robot.update_angles(angle_update, unit="deg")
        # print(f'angles: {ik_angles}')
        if SIMULATE:
            send_to_simulator(base=base, trajectory=ik_angles, vel=velPoints)

    forward_1 = np.asmatrix(forward_1)
    forward_2 = np.asmatrix(forward_2)
    forward_3 = np.asmatrix(forward_3)
    forward_4 = np.asmatrix(forward_4)

    ik_angles = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=0)
    return ik_angles.T


def map_angles_from_robot_to_simulation(angles):
    angles = angles * 180 / np.pi
    angles = np.array([angles[0], 90-angles[1], -1*angles[2], -1*angles[3]])
    # angles = np.array([0,90-27,-124,0])
    return angles


def temp_direction_to_gamma_convertion(direction):
    if direction == "top":
        return -np.pi/2
    elif direction =="bottom":
        return np.pi/2
    elif direction == "left":
        return 0
    elif direction =="right":
        return 0
    elif direction == "front":
        return 0
    else:
        return 0


def follow_path(robot, num_steps, offset, path, secondPosition=None):

    global_path = []
    global_path.append((num_steps, path))

    if use_face_star:
        filtered_path = []
        for index, point in enumerate(path):
            filtered_path.append(point)
            if index == 0:
                filtered_path.append((create_point_from_homogeneous_transform(robot.base), "top"))
            else:
                filtered_path.append(path[index-1])

        path = filtered_path
        print("NEW PATH: {}".format(path))

    global_direction = []

    update_animation = []

    save_path = None
    previous_point = None
    back_foot_pos = None
    # previous_direction = "top"
    for index, item in enumerate(path):

        direction = item[-1]
        if index == 0:
            global_direction.append((0, "top"))
            previous_direction = "top"
        else:
            global_direction.append((num_steps*index, path[index-1][-1]))
            previous_direction = path[index-1][-1]
            # previous_direction = direction

        point = list(item[0:3])
        if direction == "top":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 0.5
            point[2] = item[2] + 1
        elif direction =="bottom":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 0.5
            point[2] = item[2] + 1
        elif direction == "left":
            point[0] = item[0] - 1.37  #-1.37
            point[1] = item[1] + 0.5
            point[2] = item[2] - 0.87 #-.87
        elif direction =="right":
            point[0] = item[0] - 1.37  #-1.37
            point[1] = item[1] + 0.5
            point[2] = item[2] - 0.87 #-.87
        elif direction == "front":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 1
            point[2] = item[2] + 0.5
        elif direction =="back":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 1
            point[2] = item[2] + 0.5

        print("\t\t\n\nINDEX: {}".format(index))
        print("\nPOINT: {}   DIRECTION: {}    PREVIOUS_DIRECTION: {}".format(point, direction, path[index-1][-1]))

        if index == 0:
            # ee_up = list(point)
            ee_up = list(robot_ee_starting_point)

            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            # move_up[2] = move_up[2] + offset
            previous_angles_1, previous_angles_2, previous_angles_3 = None, None, None

            ee_up = add_offset(ee_up, previous_direction, offset)

            previous_angles_1 = move_to_point(direction, ee_up, robot, num_steps, baseID='A')
            stop_above = np.copy(point)
            stop_above = add_offset(stop_above, direction, offset)
            previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, baseID='A',
                                              previous_angles=previous_angles_1[-1].flatten().tolist()[0])
            previous_angles_3 = move_to_point(direction, point, robot, num_steps, baseID='A', previous_angles=previous_angles_2[-1].flatten().tolist()[0])

        else:
            ee_pos = robot.end_effector_position()
            initial_angles = previous_angles_3[-1].flatten().tolist()[0]
            #
            ee_pos = round_end_effector_position(ee_pos.tolist()[0], direction, previous_point)
            # ee_pos = np.copy(previous_point)


            if (index) % 2 == 0:

                new_base = flip_base(ee_pos, previous_direction, 0)

                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = False
                baseID = 'A'

            else:
                new_base = flip_base(ee_pos, previous_direction, 180)


                temp = initial_angles[1]
                initial_angles[1] = 180 / 2 + initial_angles[3]
                initial_angles[3] = temp - 180 / 2
                flip_angles = True
                baseID = 'D'

            robot.base = new_base
            robot.update_angles(initial_angles, unit="deg")

            print("Previous Point: {}".format(previous_point))
            ee_pos = back_foot_pos


            ee_up = np.copy(ee_pos)
            print("Going to point: {}\t EE Pos: {}\tRounded Pos: {}".format(point, ee_pos, ee_up))
            print("\tPrevious Direction: {}".format(previous_direction))


            # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
            if update_animation is not None:
                flatten = lambda l: [item for sublist in l for item in sublist]
                base_up = create_point_from_homogeneous_transform(update_animation[-1].robot_base).tolist()
                base_up = flatten(base_up)
                ee_up = add_offset(base_up,
                                   previous_direction, offset, path[index - 2][-1],
                                   index=index, type="ee_up")
            else:
                ee_up = add_offset(ee_up, previous_direction, offset, previous_point, index=index)

            stop_above = np.copy(point)
            stop_above = add_offset(stop_above, direction, offset)

            print("\t\t\tOffsets: \n\t\t\tEE Up: {}\n\t\t\tStop Above: {}".format(ee_up, stop_above))

            if direction == path[index - 1][-1]:
                direction = "top"
                previous_direction = "top"

            previous_angles_1, previous_angles_2, previous_angles_3 = None, None, None
            previous_angles_1 = move_to_point(previous_direction, ee_up, robot, num_steps, baseID=baseID, previous_angles=initial_angles)
            previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, baseID=baseID, previous_angles=previous_angles_1[-1].flatten().tolist()[0])
            previous_angles_3 = move_to_point(direction, point, robot, num_steps, baseID=baseID, previous_angles=previous_angles_2[-1].flatten().tolist()[0])

        save_path = update_path(save_path, previous_angles_1, previous_angles_2, previous_angles_3)
        previous_point = point
        back_foot_pos = create_point_from_homogeneous_transform(robot.base).flatten().tolist()[0]
        print("BACK FOOT: {}".format(back_foot_pos))

        if path[index-1][-1] == path[index][-1]:
            direction = path[index][-1]
        else:
            direction = path[index-1][-1]

        update_animation.append(AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], index=index, trajectory=[ee_up, stop_above, point]))

    update_animation.append(
        AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], trajectory=[],
                        index=index-1, placedObstacle=True, obstacle=[6, 0, 0]))
    return save_path, global_path, global_direction, update_animation

def update_path(save_path, new_motion_1, new_motion_2, new_motion_3):
    if save_path is None:
        return np.concatenate((new_motion_1, new_motion_2, new_motion_3))
    return np.concatenate((save_path, new_motion_1, new_motion_2, new_motion_3))

def update_path_single(save_path, new_motion_1):
    return np.concatenate((save_path, new_motion_1))

def add_offset(ee_pos, direction, offset, previous_direction=None, index=None, type=None):

    if direction == "top" or previous_direction == "top":
        ee_pos[2] = float(ee_pos[2]) + offset

    if direction == "bottom":
        ee_pos[2] = float(ee_pos[2]) - offset

    if direction == "front":
        ee_pos[1] = float(ee_pos[1]) - offset

    if direction == "back":
        ee_pos[1] = float(ee_pos[1]) + offset

    if direction == "left":
        if type == "ee_up" and previous_direction == "left":
            ee_pos[0] = float(ee_pos[0]) - (offset*2)
            ee_pos[2] = ee_pos[2] - 1.37
        else:
            ee_pos[0] = float(ee_pos[0]) - (offset*0.5)
        # if previous_point is not None:
        #     print("BACK FOOT POS: {}".format(previous_point))
        #     ee_pos[2] = previous_point[2][0]
            # ee_pos[1] = previous_point[1]
        # if index == 3:
        #     ee_pos[2] = 1
        # if index == 4:
        #     ee_pos[2] = 1
        # if index == 5:
        #     print("\n\nIndex 5")
        #     print(previous_point)
        #     print(ee_pos)
        # if index == 6:
        #     ee_pos[2] = 1
    if direction == "right":
        ee_pos[0] = float(ee_pos[0]) + offset


    return ee_pos


def check_if_point_reachable(robot, base, goal):
    delta = goal - base
    dist = np.sqrt(delta[0]**2+delta[1]**2+delta[2]**2)
    if dist > (robot.links[1].length + robot.links[2].length) * 0.95:
        raise ValueError(f'Robot cannot reach from base {base} to goal {goal}')


def send_to_simulator(base, trajectory, topic=TOPIC,vel=None):
    # base *= trotz(theta=np.pi/2)
    # print(f'before converted to pi: {trajectory}')

    position = create_point_from_homogeneous_transform(base)
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    config.pusher.trigger(u'robot', u'state', {
        u'id': topic.decode(),
        u'position': (f'{position[0]:.1f}', f'{position[1]:.1f}', f'{position[2]:.1f}', "top"),
        u'angles': [f'{trajectory[1]:.1f}', f'{-1 * trajectory[2]:.1f}', f'{-1 * trajectory[3]:.1f}'],
        u'grippers': {
            u'a': 0,
            u'd': 100
        },
        u'battery': 77,
        u'blocks_placed': 0,
        u'a_link_blocks': 1,
        u'd_link_blocks': 0,
        u'robot_state': "MOVING",
        u'end_effector_velocity': {
            u'label': current_time,
            u'value': np.linalg.norm(vel)
        }
    })
    # time.sleep(0.01)

    trajectory[0] = trajectory[0] - 90
    trajectory = trajectory * np.pi / 180
    # print(f'after converted to pi: {trajectory}')
    messagedata = AnimationUpdateMessage(robot_base=base, trajectory=trajectory)
    message_obj = MessageWrapper(topic=topic, message=messagedata)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)
    # print(f"{topic} {z}")
    socket.send_multipart([topic, z])


if __name__ == '__main__':
    SERIAL = None

    # if SERIAL:
    #     PORT = '/dev/cu.usbmodem14201'
    # else:
    #     PORT = None

    PORT = '/dev/cu.usbmodem14201'
    BAUD = 9600
    TIMEOUT = 3.0
    NUM_STEPS = 50
    path = [(1, 2, 0, "top"), (0, 2, 0, "top"), (3, 2, 3, "left"), (3, 2, 2, "left"), (3, 2, 5, "left"),
            (3, 2, 4, "left"), (3, 2, 6, "left"), (3, 2, 5, "left")]
    # path = [(1, 2, 0, "top"), (0, 2, 0, "top"), (3, 2, 3, "left"), (3, 2, 2, "left"), (2, 2, 2, "top")]
    # path = [(3, 0, 0, "top"), (1, 0, 0, "top"), (4, 0, 0, "top"),(2, 1, 0, "top"), (5, 0, 0, "top"), (1, 1, 0,
    # "top"), (0, 2, 0, "top")]
    # path = [(2, 1, 0, "top"), (1, 1, 0, "top"), (3, 1, 1, "top"), (2, 1, 0, "top"), (4, 1, 2, "top"), (3, 1, 1,
    #                                                                                                    "top"), (5,
    #                                                                                                    1, 3,
    #                                                                                                    "top"), (4,
    #                                                                                                    1, 2,
    #                                                                                                    "top"), (3,
    #                                                                                                    2, 3,
    #                                                                                                    "top"), (5,
    #                                                                                                    2, 3, "top")]
    # path = [(2, 0, 0, "top"), (1, 1, 0, "top"), (4, 1, 2, "top"), (3, 1, 1, "top"), (5, 1, 3, "top"), (4, 1, 2,
    # "top"), (3, 2, 3, "top"), (5, 2, 3, "top")]
    # path = [(3, 1, 1, "top"), (1, 1, 0, "top"), (4, 1, 2, "top"), (3, 1, 1, "top"), ]
    path = [(3, 1, 1, "top"), (2, 1, 0, "top"), (4, 1, 2, "top"), (3, 1, 1, "top"), (5, 1, 3, "top"), (4, 1, 2,
                                                                                                       "top")]

    # path = [(3, 0, 0, "top"), (1, 0, 0, "top"), (4, 0, 0, "top"), (2, 0, 0, "top"), (5, 0, 0, "top"), (3, 0, 0,
    #                                                                                                    "top")]

    blueprint = np.array([
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 1, 1]],
        [[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 1, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[1, 1, 1], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    ])

    # playground
    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    base = np.matrix([[1, 0, 0, 0.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 1.],
                      [0, 0, 0, 1]])

    robot_trajectory_serial_demo(num_steps=NUM_STEPS, baud=BAUD, serial=SERIAL, timeout=TIMEOUT, port=PORT,
                                 path=path, blueprint=blueprint, base=base)

