from datetime import datetime
from random import randint

import numpy as np

import components.robot.config as config
from components.robot.common.common import create_point_from_homogeneous_transform


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
            ee_pos[0] = float(ee_pos[0]) - (offset * 2)
            ee_pos[2] = ee_pos[2] - 1.37
        else:
            ee_pos[0] = float(ee_pos[0]) - (offset * 0.5)
    if direction == "right":
        ee_pos[0] = float(ee_pos[0]) + offset

    return ee_pos


def check_if_point_reachable(robot, base, goal):
    delta = goal - base
    dist = np.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
    if dist > (robot.links[1].length + robot.links[2].length) * 0.95:
        print(f"Delta: {delta}")
        print(f"Dist: {dist}")
        print(f"Links: {robot.links[1].length + robot.links[2].length}")
        raise ValueError(f"Robot cannot reach from base {base} to goal {goal}")


def map_angles_from_robot_to_simulation(angles):
    angles = angles * 180 / np.pi
    angles = np.array([angles[0], 90 - angles[1], -1 * angles[2], -1 * angles[3]])
    # angles = np.array([0,90-27,-124,0])
    return angles


def temp_direction_to_gamma_convertion(direction):
    if direction == "top":
        return -np.pi / 2
    elif direction == "bottom":
        return np.pi / 2
    elif direction == "left":
        return 0
    elif direction == "right":
        return 0
    elif direction == "front":
        return 0
    else:
        return 0


class AnimationUpdate:
    def __init__(
        self,
        robot,
        robot_base,
        index,
        direction,
        trajectory,
        path,
        placedObstacle=False,
        obstacle=None,
    ):
        self.robot = robot
        self.robot_base = robot_base
        self.index = index
        self.direction = direction
        self.trajectory = trajectory
        self.path = path
        self.placedObstacle = placedObstacle
        self.obstacle = obstacle


def send_to_simulator(
    base, trajectory, id=b"ROBOT_1", holding_block=None, debug_text=None
):
    import zmq
    from components.robot.communication.messages import (
        AnimationUpdateMessage,
        MessageWrapper,
    )
    import pickle
    import zlib

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect(config.communication["simulator_send_messages_port"])

    position = create_point_from_homogeneous_transform(base)
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    config.pusher.trigger(
        "robot",
        "state",
        {
            "id": id.decode(),
            "position": (
                f"{position[0]:.1f}",
                f"{position[1]:.1f}",
                f"{position[2]:.1f}",
                "top",
            ),
            "angles": [
                f"{trajectory[1]:.1f}",
                f"{-1*trajectory[2]:.1f}",
                f"{-1*trajectory[3]:.1f}",
            ],
            "grippers": {"a": 0, "d": 100},
            "battery": 77,
            "blocks_placed": 0,
            "a_link_blocks": 1,
            "d_link_blocks": 0,
            "robot_state": "MOVING",
            "end_effector_velocity": {"label": current_time, "value": randint(0, 100)},
        },
    )

    trajectory[0] = trajectory[0] - 90
    trajectory = trajectory * np.pi / 180
    # if place_block:
    #     base[2, 3] = base[2, 3] + 1
    messagedata = AnimationUpdateMessage(
        robot_base=base,
        trajectory=trajectory,
        block_on_ee=holding_block,
        debug_text=boxPrint(debug_text, 80),
    )
    message_obj = MessageWrapper(topic=id, message=messagedata)
    p = pickle.dumps(message_obj, protocol=-1)
    z = zlib.compress(p)
    socket.send_multipart([id, z])

    # time.sleep(0.01)


def breakLine(text, wrap=80):
    if len(text) > wrap:
        char = wrap
        while char > 0 and text[char] != " ":
            char -= 1
        if char:
            text = [text[:char]] + breakLine(text[char + 1 :], wrap)
        else:
            text = [text[: wrap - 1] + "-"] + breakLine(text[wrap - 1 :], wrap)
        return text
    else:
        return [cleanLine(text)]


def cleanLine(text):
    if text[-1] == " ":
        text = text[:-1]
    if text[0] == " ":
        text = text[1:]
    return text


def boxPrint(text, wrap=0):
    if text is None:
        return
    str_output = ""
    line_style = "-"
    paragraph = text.split("\n")
    if wrap > 0:
        index = 0
        while index < len(paragraph):
            paragraph[index] = cleanLine(paragraph[index])
            if len(paragraph[index]) > wrap:
                paragraph = (
                    paragraph[:index]
                    + breakLine(paragraph[index], wrap)
                    + paragraph[index + 1 :]
                )
            index += 1

    length = max([len(line) for line in paragraph])
    str_output += "\n+" + line_style * length + "+\n"
    # print('+' + line_style * length + '+')
    for line in paragraph:
        # print('|' + line + ' ' * (length - len(line)) + '|')
        str_output += line + " " * (length - len(line)) + "\n"
    # print('+' + line_style * length + '+')
    str_output += "+" + line_style * length + "+"
    return str_output
    # print(str_output)


if __name__ == "__main__":
    text = "Some text comes here to be printed in a box!!!"
    boxPrint(text, 30)
    text = "Title:\nBody lorem ipsum something body\ncheers,"
    boxPrint(text, 30)
    boxPrint(text)
    text = "No Space:\nThisIsATextThatHasNoSpaceForWrappingWhichGetsBrokenUsingDashes"
    boxPrint(text, 30)
    boxPrint(f"Base ID: {'A'}\nHolding Block: {None}\nBase: {[0.5, 1, 2]}", 80)
