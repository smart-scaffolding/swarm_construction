
##############################################################################
# Imports
##############################################################################

import py_trees
import time
import multiprocessing
import atexit
import components.robot.config as config
from components.robot.communication.messages import AnimationUpdateMessage, StatusUpdateMessagePayload, \
    StatusUpdateMessage, PlacedBlockUpdateMessagePayload, BlockLocationMessage
from components.robot.common.states import RobotBehaviors
from components.robot.common.common import create_homogeneous_transform_from_point
from components.robot.pathplanning.searches.face_star import FaceStar, BlockFace
import numpy as np
from components.robot.motionplanning.helpers import *
from components.robot.motionplanning.common import create_point_from_homogeneous_transform, flip_base, round_end_effector_position
from components.robot.motionplanning.trajectory.quintic_trajectory_planner import get_quintic_trajectory
from components.robot.motionplanning.trajectory.minimum_jerk_trajectory_planner import get_minimum_jerk_trajectory
from logzero import logger

from random import choice

##############################################################################
# Classes
##############################################################################
def remove_block(robot, block_to_pick_up, simulator_communicator, blueprint):
    """
    Action used to remove a block from the structure at the specified location

    :param robot: Robot object
    :param block_to_pick_up: Block object
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """

    try:
        logger.info(f"Robot is removing block at location {block_to_pick_up}")
        if config.SIMULATE:
            simulator_communicator.robot_communicator.send_communication("Removing block")

            logger.debug("SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS ARE BEING REMOVED")
            logger.debug(f"BLOCK: {block_to_pick_up} {block_to_pick_up.location}")
            simulator_communicator.robot_communicator.send_communication(topic=block_to_pick_up.id,
                                                                         message=BlockLocationMessage(
                block_id=block_to_pick_up.id, location=block_to_pick_up.location))

        time.sleep(0.05)
        logger.info(f"Robot has removed up block {block_to_pick_up}")


        time.sleep(0.05)
        return True
    except KeyboardInterrupt:
        pass

def place_block(robot, simulator_communicator, location_to_set_block, block, blueprint):
    """
    Action used to place a block on the structure at the specified location

    :param robot: Robot object
    :param simulator_communicator: Object to send messages to simulator
    :param location_to_set_block: Tuple denoting location and orientation to place block
    :return:
    """
    try:
        logger.info(f"Robot is placing block at location {location_to_set_block}")
        time.sleep(0.1)
        if config.SIMULATE:
            simulator_communicator.robot_communicator.send_communication(message=
                                                                         PlacedBlockUpdateMessagePayload(
                                                                             robot_base=None,
                                                                             block_placed=location_to_set_block),
                                                                        ) #TODO:

            logger.debug("SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS HAVE BEEN PLACED")
            logger.debug(f"BLOCK: {block} {block.location}")
            simulator_communicator.robot_communicator.send_communication(topic=block.id, message=BlockLocationMessage(
                block_id=block.id, location=block.next_destination))
            # Change base not to none
        logger.info(f"Robot has placed block at: {location_to_set_block}")

        time.sleep(0.1)
        return True
    except KeyboardInterrupt:
        pass

def move_to_point(direction, point, robot, num_steps, baseID, previous_angles=None, robot_id=b'ROBOT_1'):

    if baseID == 'A':
        currentEEPos = robot.DEE_POSE[:3,3]
        basePos = robot.AEE_POSE[:3,3]
    else:
        currentEEPos = robot.AEE_POSE[:3,3]
        basePos = robot.DEE_POSE[:3,3]

    check_if_point_reachable(robot, basePos, point)

    setPoints, _, _, _ = get_quintic_trajectory(points=np.array([currentEEPos,point]), set_points=num_steps)

    # setPoints, _, _, _ = get_minimum_jerk_trajectory(points=np.array([currentEEPos,point]), average_velocity=3.0,
    #                                                  frequency=100)

    # print(f"Setpoints: {setPoints}")
    logger.debug(f"Current EEPos: {currentEEPos}")
    logger.debug(f"Point: {point}")
    logger.debug(f"Base Pos: {basePos}")
    forward_1 = []
    forward_2 = []
    forward_3 = []
    forward_4 = []
    base = robot.AEE_POSE

    if baseID == 'D':
        base = robot.DEE_POSE
        # print(f"A link pos: {robot.AEE_POSE}")


    for point in setPoints:
        # ik_angles = robot.ikineConstrained(direction, p=point, flipped=flip_angles, accuracy=accuracy) * 180 / np.pi ## converted to degrees
        # print(ik_angles)
        gamma = temp_direction_to_gamma_convertion(direction)
        ik_angles = robot.ikin(goalPos=point,gamma=gamma,phi=0,baseID=baseID,simHuh=True)
        ik_angles = map_angles_from_robot_to_simulation(ik_angles)


        forward_1.append(ik_angles[0])
        forward_2.append(ik_angles[1])
        forward_3.append(ik_angles[2])
        forward_4.append(ik_angles[3])
        # print(f'each ik_angle {ik_angles}')


        if previous_angles is None:
            previous_angles = [1] * robot.length

        # if flip_angles:
        #     ik_test = np.concatenate((forward_1, forward_4, forward_3, forward_2), axis=1)
        # angle_update = ik_angles[-1].flatten().tolist()[0]
        angle_update = ik_angles
        # print(f'angle_update {angle_update}')
        # angle_update = ik_angles
        robot.update_angles(angle_update, unit="deg")
        # print(f'angles: {ik_angles}')
        send_to_simulator(base=base, trajectory=ik_angles, id=robot_id)
        robot.base = base

    forward_1 = np.asmatrix(forward_1)
    forward_2 = np.asmatrix(forward_2)
    forward_3 = np.asmatrix(forward_3)
    forward_4 = np.asmatrix(forward_4)

    # print(f'forward_1 {forward_1}')
    # print(f'forward_2 {forward_2}')
    # print(f'forward_3 {forward_3}')
    # print(f'forward_4 {forward_4}')

    ik_angles = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=0)
    return ik_angles.T


def follow_path(robot, path, place_block=False, num_steps=15, offset=1.2, secondPosition=None, robot_id=b'ROBOT_1'):

    global_path = []
    global_path.append((num_steps, path))

    global_direction = []

    update_animation = []

    save_path = None
    previous_point = None
    back_foot_pos = None
    # previous_direction = "top"
    for index, item in enumerate(path):
        previous_direction = "top"
        direction = item[-2]
        ee_to_move = item[-1]
        baseID = 'D' if robot.primary_ee == 'A' else 'A'
        # if index == 0:
        #     global_direction.append((0, "top"))
        #     previous_direction = "top"
        # else:
        #     global_direction.append((num_steps*index, path[index-1][-1]))
        #     previous_direction = path[index-1][-1]
        #     # previous_direction = direction

        point = list(item[0:3])
        if direction == "top":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 0.5
            point[2] = item[2] + 1 + (1*place_block)
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

        logger.info("\t\t\n\nINDEX: {}".format(index))
        logger.info("\nPOINT: {}   DIRECTION: {}    PREVIOUS_DIRECTION: {}".format(point, direction, path[index-1][-1]))



        # if index == 0:
        #     # ee_up = list(point)
        # ee_to_move =
        ee = robot.DEE_POSE if ee_to_move == 'D' else robot.AEE_POSE
        ee_up = list(create_point_from_homogeneous_transform(ee))
        # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
        # move_up[2] = move_up[2] + offset
        previous_angles_1, previous_angles_2, previous_angles_3 = None, None, None

        ee_up = add_offset(ee_up, previous_direction, offset)

        previous_angles_1 = move_to_point(direction, ee_up, robot, num_steps, baseID=baseID, robot_id=robot_id)
        stop_above = np.copy(point)
        stop_above = add_offset(stop_above, direction, offset)
        previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, baseID=baseID, robot_id=robot_id,
                                          previous_angles=previous_angles_1[-1].flatten().tolist()[0])
        previous_angles_3 = move_to_point(direction, point, robot, num_steps, baseID=baseID,robot_id=robot_id,
                                          previous_angles=previous_angles_2[-1].flatten().tolist()[0])

        robot.primary_ee = baseID #Switching base to ee
        # else:
        # ee_pos = robot.end_effector_position()
        # ee_pos = robot.DEE_POSE if robot.primary_ee == 'A' else robot.AEE_POSE
        # ee_pos = create_point_from_homogeneous_transform(ee_pos)
        # initial_angles = robot.get_current_joint_config(unit='deg')
        #
        # ee_pos = round_end_effector_position(ee_pos, direction, previous_point)
        #
        # if ee_to_move == robot.primary_ee:
        #     new_base = flip_base(ee_pos, previous_direction, 0)
        #     flip_angles = False
        #
        # else:
        #     new_base = flip_base(ee_pos, previous_direction, 180)
        #     flip_angles = True
        #
        # temp = initial_angles[1]
        # initial_angles[1] = 180 / 2 + initial_angles[3]
        # initial_angles[3] = temp - 180 / 2
        # baseID = ee_to_move
        # robot.primary_ee = baseID
        # robot.base = new_base
        # # ee_pos = np.copy(previous_point)
        #
        # # if (index) % 2 == 0:
        # #
        # #     new_base = flip_base(ee_pos, previous_direction, 0)
        # #
        # #     temp = initial_angles[1]
        # #     initial_angles[1] = 180 / 2 + initial_angles[3]
        # #     initial_angles[3] = temp - 180 / 2
        # #     flip_angles = False
        # #     baseID = 'A'
        # #
        # # else:
        # #     new_base = flip_base(ee_pos, previous_direction, 180)
        # #
        # #
        # #     temp = initial_angles[1]
        # #     initial_angles[1] = 180 / 2 + initial_angles[3]
        # #     initial_angles[3] = temp - 180 / 2
        # #     flip_angles = True
        # #     baseID = 'D'
        #
        # robot.base = new_base
        # robot.update_angles(initial_angles, unit="deg")
        #
        # print("Previous Point: {}".format(previous_point))
        # ee_pos = robot.DEE_POSE if baseID == 'A' else robot.AEE_POSE
        # ee_pos = create_point_from_homogeneous_transform(ee_pos)
        #
        # ee_up = np.copy(ee_pos)
        # print("Going to point: {}\t EE Pos: {}\tRounded Pos: {}".format(point, ee_pos, ee_up))
        # print("\tPrevious Direction: {}".format(previous_direction))
        #
        #
        # # TODO: FIX TO ACCEPT ANY ORIENTATION, NOT JUST +Z
        # # if update_animation is not None:
        # #     flatten = lambda l: [item for sublist in l for item in sublist]
        # #     base_up = create_point_from_homogeneous_transform(update_animation[-1].robot_base).tolist()
        # #     base_up = flatten(base_up)
        # #     ee_up = add_offset(base_up,
        # #                        previous_direction, offset, path[index - 2][-1],
        # #                        index=index, type="ee_up")
        # # else:
        # ee_up = add_offset(ee_up, previous_direction, offset, previous_point, index=index)
        #
        # stop_above = np.copy(point)
        # stop_above = add_offset(stop_above, direction, offset)
        #
        # print("\t\t\tOffsets: \n\t\t\tEE Up: {}\n\t\t\tStop Above: {}".format(ee_up, stop_above))
        #
        # if direction == path[index - 1][-1]:
        #     direction = "top"
        #     previous_direction = "top"
        #
        # previous_angles_1, previous_angles_2, previous_angles_3 = None, None, None
        # previous_angles_1 = move_to_point(previous_direction, ee_up, robot, num_steps, baseID=baseID, previous_angles=initial_angles)
        # previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, baseID=baseID, previous_angles=previous_angles_1[-1].flatten().tolist()[0])
        # previous_angles_3 = move_to_point(direction, point, robot, num_steps, baseID=baseID, previous_angles=previous_angles_2[-1].flatten().tolist()[0])

        save_path = update_path(save_path, previous_angles_1, previous_angles_2, previous_angles_3)
        previous_point = point
        # back_foot_pos = create_point_from_homogeneous_transform(robot.base).flatten().tolist()[0]
        # print("BACK FOOT: {}".format(back_foot_pos))

        if path[index-1][-1] == path[index][-1]:
            direction = path[index][-1]
        else:
            direction = path[index-1][-1]

        update_animation.append(AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], index=index, trajectory=[ee_up, stop_above, point]))

    update_animation.append(
        AnimationUpdate(robot=robot, robot_base=robot.base, direction=direction, path=path[index:], trajectory=[],
                        index=index-1, placedObstacle=True, obstacle=[6, 0, 0]))
    return robot



def get_path_to_point(robot, current_position, destination, simulator_communicator, blueprint, old_path=None):
    """
    Returns a path to the specified point

    :param robot: Robot object
    :param destination: Tuple consisting of 3d location and face to reach
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """
    logger.debug("Getting a path to return")
    # time.sleep(0.05)
    # print("Path Path to Traversefound")
    # if config.SIMULATE:
    #     simulator_communicator.robot_communicator.send_communication(message="Got path")
    # armReach = [2.38, 1.58]

    armReach = [1.5, 1.5]

    # armReach = [1.5, 1.5]
    # blueprint = np.array([
    #                          [[1] * 1] * 12,
    #                      ] * 12)

    faceStarPlanner = FaceStar(blueprint, armReach)
    # start_face = choice([BlockFace(1, 1, 0, "top"), BlockFace(4, 1, 0, "top"), BlockFace(7, 1, 0, "top"),
    #         BlockFace(1, 4, 0, "top"), BlockFace(4, 4, 0, "top"), BlockFace(5, 4, 0, "top"),
    #         BlockFace(1, 7, 0, "top"), BlockFace(3, 7, 0, "top"), BlockFace(5, 7, 0, "top"),
    #         ])
    try:
        direction = destination[3]
    except IndexError:
        logger.warning("No destination specified, setting to default 'top'")
        direction = 'top'

    point = list(destination)

    #TODO: Have conversion for other directions too
    modified_det = np.array([point[0]+0.5, point[1]+0.5, point[2]])
    d_pose = np.array(create_point_from_homogeneous_transform(robot.DEE_POSE))
    a_pose = np.array(create_point_from_homogeneous_transform(robot.AEE_POSE))

    # print(type(d_pose))
    if (np.linalg.norm(d_pose - modified_det) <= 1.2):
        return []

    if (np.linalg.norm(a_pose - modified_det) <= 1.2):
        return []

    if direction == "top" or direction == "bottom":
        point[0] = round(point[0])
        point[1] = round(point[1])
        point[2] = point[2]-1
    if direction == "left" or direction == "right":
        point[0] = point[0] + 1.37  # -1.37
        point[1] = point[1] - 0.5
        point[2] = point[2] + 0.87  # -.87
    if direction == "front" or direction == "back":
        point[0] = point[0] - 0.5
        point[1] = point[1] - 1
        point[2] = point[2] - 0.5
    logger.debug(f"Going to point: {point}")



    try:
        path = faceStarPlanner.get_path(start=current_position, goal=BlockFace(point[0], point[1],
                                                                                        point[2],
                                                                                        direction, robot.primary_ee))
    except Exception as e:
        logger.exception(e)
        logger.error(f"Path planner unable to find path to {point}")


    # return [choice([(1, 0, 0, "Top"), (2, 0, 0, "Top"), (3, 0, 0, "Top"), (4, 0, 0, "Top")])]
    # path.pop(0)
    logger.debug(f"Path: {path}")
    return path
    # return []

class RemoveBlock(py_trees.behaviour.Behaviour):
    """
    Robot behavior to remove a block from the structure. Upon receiving a message
    to remove a block, the behavior will perform the necessary actions to remove
    the block from the structure.

    Note this behavior is blocking in that it will wait for the robot to either
    succeed or fail at removing the block. This blocking is performed to ensure
    that the robot cannot be interrupted when in the process of removing a
    block, as all blocks should be either removed completely or not at all

    For each tick, the behavior returns:
        SUCCESS: If robot has successfully removed block
        FAILURE: Robot has failed at removing a block
    """

    def __init__(self, robot, key, robot_communicator, simulator_communicator, blueprint, name="RemoveBlock",
                    remove_block_key="remove_block/block_to_remove",
                    blueprint_key="state/blueprint",

                 ):
        """

        :param robot:
        :param key:
        :param robot_communicator:
        :param simulator_communicator:
        :param name:
        """
        super(RemoveBlock, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.robot = robot
        self.blackboard = self.attach_blackboard_client("State", "remove_block")
        self.state = self.attach_blackboard_client()
        self.key = key
        self.blueprint = blueprint
        self.keys = {
            "remove_block_key": remove_block_key,
            "blueprint": blueprint_key,
        }

        self.blackboard.register_key(key=str(self.key), access=py_trees.common.Access.READ)
        self.state.register_key(key=blueprint_key, access=py_trees.common.Access.WRITE)
        # self.blackboard.register_key(key=remove_block_key, access=py_trees.common.Access.READ)
        self.robot_communicator = robot_communicator
        self.simulator_communicator = simulator_communicator

    def setup(self):
        # self.parent_connection, self.child_connection = multiprocessing.Pipe()
        # self.remove_block_action = multiprocessing.Process(target=remove_block, args=(self.child_connection, self.robot,
        #                                                           self.block_to_remove))
        # atexit.register(self.remove_block_action.terminate)
        # self.remove_block_action.start()
        pass

    def initialise(self):
        self.percentage_completion = 0
        self.block_to_remove = self.blackboard.get(str(self.key))
        self.blueprint = self.state.get(name=self.keys["blueprint"])
        # self.block = self.blackboard.get(self.keys["remove_block_key"])

        # print(self.blackboard)

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        # print(self.blackboard)
        # self.block_to_remove = self.blackboard.get(str(self.key))
        # self.block = self.blackboard.get(self.keys["remove_block_key"])
        # if self.parent_connection.poll():
        #     self.percentage_completion = self.parent_connection.recv().pop()
        #     if self.percentage_completion == 100:
        #         new_status = py_trees.common.Status.SUCCESS
        remove_block_action = remove_block(self.robot, self.block_to_remove, self.simulator_communicator,
                                           blueprint=self.blueprint)
        if remove_block_action is True:
            new_status = py_trees.common.Status.SUCCESS
            # print(f"Before error {self.block_to_remove}")
            self.blueprint[self.block_to_remove.location] = 0
            self.state.set(name=self.keys["blueprint"], value=self.blueprint)
        else:
            new_status = py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = f"Robot has finished removing block at location {self.block_to_remove}"
            self.logger.debug(
                "%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        else:
            self.feedback_message = f"Robot has failed to remove block at location {self.block_to_remove}"
            self.logger.debug("%s.update()[%s][%s]" % (self.__class__.__name__, self.status, self.feedback_message))
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class PlaceBlock(py_trees.behaviour.Behaviour):
    """
    Robot behavior to place a block on the structure. Upon receiving a message
    to place a block, the behavior will perform the necessary actions to place
    the block on the structure.

    Note this behavior is blocking in that it will wait for the robot to either
    succeed or fail at placing the block. This blocking is performed to ensure
    that the robot cannot be interrupted when in the process of placing a
    block, as all blocks should be either placed completely or not at all

    For each tick, the behavior returns:
        SUCCESS: If robot has successfully placed block
        FAILURE: Robot has failed at placing a block
    """

    def __init__(self, robot, key, robot_communicator, simulator_communicator, blueprint,
                 state_key="state/block_has_been_placed",
                 place_block_key="place_block/location_to_place_block",
                 blocks_to_move_key="state/blocks_to_move",
                 remove_block_key="remove_block/block_to_remove",
                 blueprint_key="state/blueprint",
                 name="PlaceBlock"):
        """
        Default construction.
        """
        super(PlaceBlock, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.robot = robot
        self.key = key
        self.state_key = state_key
        self.blackboard = self.attach_blackboard_client("State", "place_block")
        self.blackboard.register_key(key=str(key), access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=remove_block_key, access=py_trees.common.Access.READ)
        self.blueprint = blueprint
        self.state = self.attach_blackboard_client()
        self.state.register_key(key=state_key, access=py_trees.common.Access.WRITE)
        self.keys = {
            "place_block_key": place_block_key,
            "robot_state": state_key,
            "remove_block_key": remove_block_key,
            "blueprint": blueprint_key,
            "blocks_to_move_key": blocks_to_move_key,
        }

        # self.blackboard.register_key()

        self.state.register_key(key=self.keys["place_block_key"], access=py_trees.common.Access.READ)
        self.state.register_key(key=self.keys["remove_block_key"], access=py_trees.common.Access.READ)
        self.state.register_key(key=blueprint_key, access=py_trees.common.Access.WRITE)

        self.state.register_key(key=blocks_to_move_key, access=py_trees.common.Access.WRITE)
        self.robot_communicator = robot_communicator
        self.simulator_communicator = simulator_communicator

    def setup(self):
        """
        No delayed initialisation required for this example.
        """
        # self.parent_connection, self.child_connection = multiprocessing.Pipe()
        # self.remove_block_action = multiprocessing.Process(target=remove_block, args=(self.child_connection,
        # self.robot,
        #                                                           self.block_to_remove))
        # atexit.register(self.remove_block_action.terminate)
        # self.remove_block_action.start()
        pass

    def initialise(self):
        """
        Reset a counter variable.
        """
        self.percentage_completion = 0
        # self.location_to_place_block = self.blackboard.get(str(self.key))
        blocks_to_move_key = self.keys["place_block_key"]
        self.location_to_place_block = self.state.get(blocks_to_move_key)
        self.move_block = self.state.get(self.keys["remove_block_key"])
        self.blueprint = self.state.get(name=self.keys["blueprint"])
        self.blocks_to_move = self.state.get(name=self.keys["blocks_to_move_key"])

    def update(self):
        """
        Increment the counter and decide upon a new status result for the behaviour.
        """
        new_status = py_trees.common.Status.RUNNING
        # if self.parent_connection.poll():
        #     self.percentage_completion = self.parent_connection.recv().pop()
        #     if self.percentage_completion == 100:
        #         new_status = py_trees.common.Status.SUCCESS
        place_block_action = place_block(robot=self.robot, location_to_set_block=self.location_to_place_block,
                                         simulator_communicator=self.simulator_communicator, block=self.move_block,
                                         blueprint=self.blueprint)

        if place_block_action is True:
            new_status = py_trees.common.Status.SUCCESS
            self.blueprint[self.location_to_place_block] = 1
            self.state.set(name=self.keys["blueprint"], value=self.blueprint)
            self.blocks_to_move.pop()
            self.state.set(name=self.keys["blocks_to_move_key"], value=self.blocks_to_move)
        else:
            new_status = py_trees.common.Status.FAILURE

        if new_status == py_trees.common.Status.SUCCESS:
            # self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
            #                                                                   message=AnimationUpdateMessage(
            #                                                                       robot_base=create_homogeneous_transform_from_point([0.5, 0.5, 1]),
            #                                                                           obstacle=self.location_to_place_block))
            #TODO: Change base so it is not none (use blackboard)
            self.robot_communicator.robot_communicator.send_communication(topic=self.robot,
                                                                          message=StatusUpdateMessage(
                                                                              status=RobotBehaviors.MOVE,
                                                                              payload=PlacedBlockUpdateMessagePayload(
                                                                                  robot_base=None,
                                                                                  block_placed=self.location_to_place_block)))



            self.state.set(name=self.state_key, value=True)
            self.feedback_message = f"Robot has finished placing block at location {self.location_to_place_block}"
            self.logger.debug(
                "%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        else:
            self.feedback_message = f"Robot has failed to place block at location {self.location_to_place_block}"
            self.logger.debug("%s.update()[%s][%s]" % (self.__class__.__name__, self.status, self.feedback_message))
        return new_status

class NavigateToPoint(py_trees.behaviour.Behaviour):
    """
    Robot behavior to navigate to a specific point in the structure. Upon receiving
    a message to reach the desired destination, the behavior will perform the
    necessary actions to reach the sepcified point.

    Note that the behavior does not do any of the work, it alerts a separate process
    to do the real work. This behavior is not blocking and can be interrupted in
    the middle of trying to navigate to a point.

    Note that path planning is performed for each initialization. May want to remove this

    For each tick, the behavior returns:
        SUCCESS: If robot has reached specified point
        RUNNING: Robot in process of reaching specified point but has not yet reached it
        FAILURE: Robot has failed to reach point
    """

    def __init__(self, robot, key, robot_communicator, simulator_communicator, blueprint, name="NavigateToPoint",
                 current_position_key="current_position",
                 robot_model_key="robot",
                 robot_status_key="robot_status",
                 blueprint_key="blueprint"):
        """

        :param robot:
        :param key:
        :param robot_communicator:
        :param simulator_communicator:
        :param name:
        """
        super(NavigateToPoint, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.robot_id = robot
        # self.point_to_reach = destination
        self.blackboard = self.attach_blackboard_client("Navigation", "navigation")
        self.state = self.attach_blackboard_client("State", "state")
        self.key = key
        self.keys = {
            "blueprint": blueprint_key,
            "robot_status": robot_status_key
        }
        self.current_position_key = current_position_key
        self.robot_model_key = robot_model_key
        self.blackboard.register_key(key=self.key, access=py_trees.common.Access.READ)
        self.state.register_key(key=self.key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=current_position_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=robot_model_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=blueprint_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=robot_status_key, access=py_trees.common.Access.READ)

        self.blueprint = blueprint
        self.robot_communicator = robot_communicator
        self.simulator_communicator = simulator_communicator

    def setup(self):
        # self.parent_connection, self.child_connection = multiprocessing.Pipe()

        # self.ik_planner = multiprocessing.Process(target=ik_solver, args=(self.child_connection, self.robot,
        #                                                                   self.simulator_communicator, self.blueprint))
        # atexit.register(self.ik_planner.terminate)
        # self.ik_planner.start()
        pass

    def initialise(self):
        self.reached_point = (True, None)
        self.point_to_reach = self.blackboard.get(self.key)
        self.blueprint = self.state.get(name=self.keys["blueprint"])
        self.current_position = self.state.get(self.current_position_key)
        logger.info(f"[{self.name.upper()}]: Current Position: ({self.current_position.xPos}, "
                f"{self.current_position.yPos}, {self.current_position.zPos}, {self.current_position.face})")
        logger.info(f"[{self.name.upper()}]: Got point to reach: {self.point_to_reach}")
        self.robot = self.state.get(self.robot_model_key)

        self.path = get_path_to_point(self.robot, current_position=self.current_position, destination=self.point_to_reach,
                                      simulator_communicator=self.robot_communicator,
                                      blueprint=self.blueprint)

        # print(self.blackboard)
    def update(self):
        logger.debug(f"[{self.name.upper()}]: In Navigate to Point")
        new_status = py_trees.common.Status.RUNNING
        # try:
        #     self.point_to_reach = self.blackboard.get(str(self.key))
        # except KeyError:
        #     return py_trees.common.Status.FAILURE

        # self.percentage_completion = 0
        if len(self.path) > 0 and self.reached_point[0]:
            self.next_point = self.path.pop(0)



            place_block=False
            modified_goal = np.array(self.point_to_reach)
            modified_goal[2] -= 1

            robot_status = self.state.get(name=self.keys["robot_status"])
            if(np.linalg.norm(modified_goal - np.array(self.next_point[0:3])) <= 0.6 and (robot_status ==
                                                                                          RobotBehaviors.BUILD or
                                                                                          robot_status ==
                                                                                          RobotBehaviors.FERRY)):
                place_block = True

            self.robot = follow_path(self.robot, [self.next_point], robot_id=self.robot_id, place_block=place_block)
            self.state.set(name=self.robot_model_key, value=self.robot)
            base = create_point_from_homogeneous_transform(
                self.robot.base)

            base_block_face = BlockFace(base[0], base[1], base[2],
                      'top', self.robot.primary_ee)


            self.state.set(name=self.current_position_key, value=base_block_face)

            logger.debug(f"ROBOT BASE: {self.robot.base}")
            logger.debug(f"Next point moving to: {self.next_point}")
            # self.parent_connection.send(self.next_point)
            # base = create_homogeneous_transform_from_point(np.array(self.next_point[0:3]))
            # self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
            #                                                                   message=AnimationUpdateMessage(
            #                                                                       robot_base=base))
            # self.robot_communicator.robot_communicator.send_communication(topic=self.robot,
            #                                                               message=StatusUpdateMessage(
            #                                                                   status=RobotBehaviors.MOVE,
            #                                                                   payload=StatusUpdateMessagePayload(
            #                                                                     robot_base=base)))

        # if self.parent_connection.poll():
        #     self.percentage_completion = self.parent_connection.recv().pop()
        #     if self.percentage_completion == "FAILURE":
        #         print(f"[{self.name.upper()}]: IK solver unable to find solution")
        #         return py_trees.common.Status.FAILURE
        #     while self.percentage_completion != 100:
        #         """
        #         Note will block other behaviors until done moving, this is to ensure
        #         that while robot detaches foot and moves it to new location, it can
        #         never be interrupted. Otherwise, robot could be stopped with foot hanging
        #         in midair.
        #         """
        #         point = np.array(self.next_point[0:3])
        #         try:
        #             z = point[2] + 1
        #         except IndexError:
        #             z = 4
        #
        #         # TODO: Fix last point
        #         base = create_homogeneous_transform_from_point((point[0] + 0.5, point[1] + 0.5, z))
        #         # base = create_homogeneous_transform_from_point(np.array(self.next_point[0:3]))
        #         self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
        #                                                              message=AnimationUpdateMessage(
        #             robot_base=base, path=self.path, trajectory=np.array([[1.61095456e-15,  6.18966422e+01,
        #                                                                    -1.23793284e+02, -2.80564688e+01]])*np.pi/180))
        #
        #         self.robot_communicator.robot_communicator.send_communication(topic=self.robot,
        #                                                                       message=StatusUpdateMessage(
        #                                                                           status=RobotBehaviors.MOVE,
        #                                                                           payload=StatusUpdateMessagePayload(
        #                                                                               robot_base=base)))
        #         self.percentage_completion = self.parent_connection.recv().pop()
        #         self.reached_point = (False, self.next_point)
        #         self.state.set(name=self.current_position_key, value=BlockFace(self.next_point[0], self.next_point[
        #             1], self.next_point[2], self.next_point[3]))
        #         print(f"Updated Current position to: {self.state.get(self.current_position_key).return_tuple()}")
        #     if self.percentage_completion == 100:
        #         print(f"[{self.name.upper()}]: Reached point: {self.next_point}")
        #         self.reached_point = (True, self.next_point)
        #         self.state.set(name=self.current_position_key, value=BlockFace(self.next_point[0], self.next_point[
        #             1], self.next_point[2], self.next_point[3]))
        #         print(f"Updated Current position to: {self.state.get(self.current_position_key).return_tuple()}")
        # else:
        #     self.reached_point = (True, self.reached_point[1]) #TODO: Set to false, need to wait for all points to
        #     # finish
        #     try:
        #         point = np.array(self.next_point[0:3])
        #         direction = self.next_point[3]
        #     except AttributeError:
        #         point = np.array(self.point_to_reach[0:3])
        #         try:
        #             direction = self.point_to_reach[3]
        #         except IndexError:
        #             direction = "top"
        #     try:
        #         z = point[2] + 1
        #     except IndexError:
        #         z = 4
        #
        #     #TODO: Fix last point
        #     base = create_homogeneous_transform_from_point((point[0]+0.5, point[1]+0.5, z))
        #     # self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
        #     #                                                                   message=AnimationUpdateMessage(
        #     #                                                                       robot_base=base))
        #
        #     self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
        #                                                                       message=AnimationUpdateMessage(
        #                                                                           robot_base=base, path=self.path,
        #                                                                           trajectory=np.array(
        #                                                                               [[1.61095456e-15, 6.18966422e+01,
        #                                                                                 -1.23793284e+02,
        #                                                                                 -2.80564688e+01]])*np.pi/180))
        #     self.state.set(name=self.current_position_key, value=BlockFace(point[0], point[
        #         1], point[2], direction))
        #     print(f"Updated Current position to: {self.state.get(self.current_position_key).return_tuple()}")
        if len(self.path) <= 0 and self.reached_point[0]:
            new_status = py_trees.common.Status.SUCCESS

        # new_status = py_trees.common.Status.SUCCESS
        if new_status == py_trees.common.Status.SUCCESS:
            self.state.set(name=self.key, value=True)
            self.feedback_message = "Reached destination"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        else:
            if self.reached_point[0]:
                self.feedback_message = f"Reached point: {self.reached_point[1]}"
            else:
                self.feedback_message = f"(Still) Moving to point ({self.reached_point[1]})"
            self.logger.debug("%s.update()[%s][%s]" % (self.__class__.__name__, self.status, self.feedback_message))
        return new_status

    def terminate(self, new_status):
        """
        Nothing to clean up in this example.
        """
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

def create_root(robot, robot_communicator, simulator_communicator, blueprint, set_variables=False):
    """
    Create the behavior tree made up of the following behaviors:

    NavigateToPoint, RemoveBlock, NavigateToPoint, PlaceBlock

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :param set_variables: Used for testing purposes, can set blackboard variables to test with
    :return:
    """

    task_one = NavigateToPoint(name="MoveToBlockToRemove", key="point_to_reach", robot=robot,
                               robot_communicator=robot_communicator, simulator_communicator=simulator_communicator,
                               blueprint=blueprint)
    task_two = RemoveBlock(name="RemoveBlock", key="block_to_remove", robot=robot,
                           robot_communicator=robot_communicator, simulator_communicator=simulator_communicator, blueprint=blueprint)
    task_three = NavigateToPoint(name="MoveToPlaceBlock", key="point_to_reach_2", robot=robot,
                                 robot_communicator=robot_communicator, simulator_communicator=simulator_communicator, blueprint=blueprint)
    task_four = PlaceBlock(name="PlaceBlock", key="location_to_place_block", robot=robot,
                           robot_communicator=robot_communicator, simulator_communicator=simulator_communicator, blueprint=blueprint)

    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off",
        tasks=[task_one, task_two, task_three, task_four]
    )

    root = py_trees.composites.Selector(name="Root")

    if set_variables:
        set_blackboard_variable = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination", variable_name="navigation/point_to_reach", variable_value=(6, 6, 6, "Left")
        )

        set_blackboard_variable1 = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination", variable_name="place_block/location_to_place_block", variable_value=(4, 4, 4, "Left")
        )

        set_blackboard_variable2 = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination", variable_name="navigation/point_to_reach_2", variable_value=(0, 0, 0, "Left")
        )

        set_blackboard_variable3 = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination", variable_name="remove_block/block_to_remove", variable_value=(9, 9, 9, "Left")
        )

        # write_blackboard_variable = BlackboardWriter(name="Writer")
        check_blackboard_variable = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check set variable", variable_name="asdf", expected_value=str((6, 6, 6))
        )
        mid = py_trees.composites.Sequence("Blackboard Demo")
        mid.add_children([
            set_blackboard_variable,
            set_blackboard_variable1,
            set_blackboard_variable2,
            set_blackboard_variable3,
            check_blackboard_variable,
        ])
        root.add_children([mid, piwylo])
    else:
        root.add_children([piwylo])

    return root


##############################################################################
# Main
##############################################################################



def main():
    py_trees.logging.level = py_trees.logging.Level.INFO
    root = create_root(set_variables=True, robot="MyRobot")



    ####################
    # Tree Stewardship
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(root)

    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    # py_trees.console.read_single_keypress()
    for unused_i in range(1, 25):
        try:
            behaviour_tree.tick()
            print("Tree is ticking")
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

def get_motion_planning_behaviors_tree(robot_communicator, simulator_communicator, robot, blueprint):
    """

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """
    return create_root(robot_communicator=robot_communicator, simulator_communicator=simulator_communicator,
                       robot=robot, blueprint=blueprint)

def get_move_to_point_tree(robot_communicator, simulator_communicator, robot, blueprint):
    """

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """

    task_one = NavigateToPoint(name="MoveToBlockToRemove", key="point_to_reach", robot=robot,
                               robot_communicator=robot_communicator, simulator_communicator=simulator_communicator,
                               blueprint=blueprint)
    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off",
        tasks=[task_one]
    )

    root = py_trees.composites.Selector(name="Root")
    root.add_children([piwylo])

    return root

if __name__ == '__main__':
    main()