##############################################################################
# Imports
##############################################################################

import time

import py_trees
from logzero import logger

from components.robot.common.states import RobotBehaviors, BlockMoved, PathPlanners
from components.robot.communication.messages import (
    StatusUpdateMessage,
    PlacedBlockUpdateMessagePayload,
)
from components.robot.motionplanning.common import create_point_from_homogeneous_transform
from components.robot.motionplanning.helpers import *
from components.robot.motionplanning.trajectory.quintic_trajectory_planner import (
    get_quintic_trajectory,
)
from components.robot.pathplanning.path_planner import PathPlanner
from components.robot.pathplanning.searches.face_star import FaceStar, BlockFace


##############################################################################
# Classes
##############################################################################
def remove_block(
    robot, block_to_pick_up, simulator_communicator, structure_communicator, blueprint
):
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

            logger.debug(
                "SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS ARE BEING REMOVED"
            )
            logger.debug(f"BLOCK: {block_to_pick_up} {block_to_pick_up.location}")
            # simulator_communicator.robot_communicator.send_communication(
            #     topic=block_to_pick_up.id,
            #     message=BlockLocationMessage(
            #         block_id=block_to_pick_up.id, location=block_to_pick_up.location
            #     ),
            # )
        # structure_communicator.robot_communicator.send_communication(topic=block_to_pick_up.id,
        #                                                              message=BlockLocationMessage(
        #                                                                  block_id=block_to_pick_up.id,
        #                                                                  location=block_to_pick_up.location,
        #                                                                  removed=True))
        # time.sleep(0.05)
        logger.info(f"Robot has removed up block {block_to_pick_up}")

        # time.sleep(0.05)
        return True
    except KeyboardInterrupt:
        pass


def place_block(
    robot,
    simulator_communicator,
    location_to_set_block,
    block,
    structure_communicator,
    blueprint,
):
    """
    Action used to place a block on the structure at the specified location

    :param robot: Robot object
    :param simulator_communicator: Object to send messages to simulator
    :param location_to_set_block: Tuple denoting location and orientation to place block
    :return:
    """
    try:
        logger.info(f"Robot is placing block at location {location_to_set_block}")
        # time.sleep(0.1)
        if config.SIMULATE:
            simulator_communicator.robot_communicator.send_communication(
                message=PlacedBlockUpdateMessagePayload(
                    robot_base=None, block_placed=location_to_set_block
                ),
            )  # TODO:

            logger.debug(
                "SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS HAVE BEEN PLACED"
            )
            logger.debug(f"BLOCK: {block} {block.location}")
            # simulator_communicator.robot_communicator.send_communication(
            #     topic=block.id,
            #     message=BlockLocationMessage(
            #         block_id=block.id, location=block.next_destination
            #     ),
            # )
            # Change base not to none

        # structure_communicator.robot_communicator.send_communication(topic=block.id, message=BlockLocationMessage(
        #         block_id=block.id, location=block.next_destination, removed=False))
        logger.info(f"Robot has placed block at: {location_to_set_block}")

        # time.sleep(0.1)
        return True
    except KeyboardInterrupt:
        pass


def move_to_point(
    direction,
    point,
    robot,
    num_steps,
    baseID,
    previous_angles=None,
    robot_id=b"ROBOT_1",
    place_block=0,
    block_on_ee=None,
):
    """

    :param direction:
    :param point:
    :param robot:
    :param num_steps:
    :param baseID:
    :param previous_angles:
    :param robot_id:
    :param place_block:
    :param block_on_ee:
    :return:
    """
    if baseID == "A":
        currentEEPos = robot.DEE_POSE[:3, 3]
        basePos = robot.AEE_POSE[:3, 3]
    else:
        currentEEPos = robot.AEE_POSE[:3, 3]
        basePos = robot.DEE_POSE[:3, 3]

    check_if_point_reachable(robot, basePos, point)

    setPoints, _, _, _ = get_quintic_trajectory(
        points=np.array([currentEEPos, point]), set_points=num_steps
    )

    # setPoints, _, _, _ = get_minimum_jerk_trajectory(points=np.array([currentEEPos,point]), average_velocity=3.0,
    #                                                  frequency=100)

    # print(f"Setpoints: {setPoints}")
    logger.debug(f"Current EEPos: {currentEEPos}")
    logger.debug(f"Point: {point}")
    logger.debug(f"Base Pos: {basePos}")

    if (
        currentEEPos[0] == basePos[0]
        and currentEEPos[1] == basePos[1]
        and currentEEPos[2] == basePos[2]
    ):
        logger.exception(
            "End effector and base are at the same position, this is something wrong"
        )
        raise Exception(
            "End effector and base are at the same position, this is something wrong"
        )
    forward_1 = []
    forward_2 = []
    forward_3 = []
    forward_4 = []
    base = robot.AEE_POSE

    if baseID == "D":
        base = robot.DEE_POSE
        # print(f"A link pos: {robot.AEE_POSE}")

    for point in setPoints:
        gamma = temp_direction_to_gamma_convertion(direction)
        ik_angles = robot.ikin(
            goalPos=point,
            gamma=gamma,
            phi=0,
            baseID=baseID,
            simHuh=True,
            placeBlock=place_block,
        )
        ik_angles = map_angles_from_robot_to_simulation(ik_angles)

        forward_1.append(ik_angles[0])
        forward_2.append(ik_angles[1])
        forward_3.append(ik_angles[2])
        forward_4.append(ik_angles[3])

        angle_update = ik_angles

        robot.update_angles(angle_update, unit="deg")
        send_to_simulator(
            base=base,
            trajectory=ik_angles,
            id=robot_id,
            holding_block=block_on_ee,
            debug_text=f"Base ID: {baseID}\nHolding Block: {block_on_ee}\nBase: {base[0:3, 3]}",
        )
        # robot.base = base

    forward_1 = np.asmatrix(forward_1)
    forward_2 = np.asmatrix(forward_2)
    forward_3 = np.asmatrix(forward_3)
    forward_4 = np.asmatrix(forward_4)

    ik_angles = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=0)
    return ik_angles.T


def follow_path(
    robot,
    path,
    blueprint,
    place_block=0,
    num_steps=5,
    offset=1.2,
    robot_id=b"ROBOT_1",
    block_on_ee=None,
):
    """

    :param robot:
    :param path:
    :param blueprint:
    :param place_block:
    :param num_steps:
    :param offset:
    :param robot_id:
    :param block_on_ee:
    :return:
    """
    for index, item in enumerate(path):
        previous_direction = "top"
        direction = item[-2]
        direction = "top"  # TODO: REMOVE ME
        ee_to_move = item[-1]
        baseID = "D" if robot.primary_ee == "A" else "A"

        if ee_to_move != robot.primary_ee:
            logger.debug(
                f"EE to move ({ee_to_move}) != robot primary ee ({robot.primary_ee})"
            )
            logger.debug(f"Returning")
            robot.primary_ee = "D" if robot.primary_ee == "A" else "A"
            return robot, False
        if place_block == 1 and block_on_ee is None:
            logger.exception(
                f"Was told to place block, but there is no block on end effector"
            )
            logger.exception(f"Returning")
            # robot.primary_ee = "D" if robot.primary_ee == "A" else "A"
            # return robot
        # logger.debug(f"Robot A Link: {robot.links[0].d} {robot.links[0].length}")
        # logger.debug(f"Robot D Link: {robot.links[3].a} {robot.links[3].length}")
        point = list(item[0:3])
        if direction == "top":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 0.5

            if place_block:
                point[2] = item[2] + 2
            elif block_on_ee:
                point[2] = item[2] + 2
            else:
                point[2] = item[2] + 1

        elif direction == "bottom":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 0.5
            point[2] = item[2] + 1
        elif direction == "left":
            point[0] = item[0] - 1.37  # -1.37
            point[1] = item[1] + 0.5
            point[2] = item[2] - 0.87  # -.87
        elif direction == "right":
            point[0] = item[0] - 1.37  # -1.37
            point[1] = item[1] + 0.5
            point[2] = item[2] - 0.87  # -.87
        elif direction == "front":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 1
            point[2] = item[2] + 0.5
        elif direction == "back":
            point[0] = item[0] + 0.5
            point[1] = item[1] + 1
            point[2] = item[2] + 0.5

        logger.info("\t\t\n\nINDEX: {}".format(index))
        logger.info(
            "\nPOINT: {}   DIRECTION: {}    PREVIOUS_DIRECTION: {}".format(
                point, direction, path[index - 1][-1]
            )
        )

        ee = robot.DEE_POSE if ee_to_move == "D" else robot.AEE_POSE
        ee_up = list(create_point_from_homogeneous_transform(ee))
        if np.linalg.norm(np.array(point) - np.array(ee_up)) <= 0.01:
            logger.debug("No sense in moving, destination is where I am already at")
            logger.debug(f"Point I am going to: {point}, point my ee is at: {ee_up}")
            robot.primary_ee = "D" if robot.primary_ee == "A" else "A"
            return robot, True

        ee_up = add_offset(ee_up, previous_direction, offset)

        previous_angles_1 = move_to_point(
            direction,
            ee_up,
            robot,
            num_steps,
            baseID=baseID,
            robot_id=robot_id,
            place_block=place_block,
            block_on_ee=block_on_ee,
        )
        stop_above = np.copy(point)
        stop_above = add_offset(stop_above, direction, offset)

        if baseID == "A":
            move_base = np.copy(robot.AEE_POSE[:3, 3])
        else:
            move_base = np.copy(robot.DEE_POSE[:3, 3])

        move_base = [int(x) for x in move_base]
        modified_blueprint = np.copy(blueprint)

        pad_x_before = 3
        pad_x_after = 3
        pad_y_before = 3
        pad_y_after = 3
        pad_z_before = 0
        pad_z_after = 3
        modified_blueprint = np.pad(
            modified_blueprint,
            (
                (pad_x_before, pad_x_after),
                (pad_y_before, pad_y_after),
                (pad_z_before, pad_z_after),
            ),
            "constant",
        )
        modified_blueprint = np.logical_not(modified_blueprint)
        modified_blueprint[move_base[0] + pad_x_before][move_base[1] + pad_y_before] = 0
        planner = PathPlanner(
            blueprint=modified_blueprint, arm_reach=(1, 1), search=PathPlanners.AStar
        )

        start = ee_up
        start[0] = int(round(start[0] - 0.5 + pad_x_before))
        start[1] = int(round(start[1] - 0.5 + pad_y_before))
        start[2] = int(round(start[2] - offset))

        goal = stop_above
        goal[0] = int(round(goal[0] - 0.5 + pad_x_before))
        goal[1] = int(round(goal[1] - 0.5 + pad_y_before))
        goal[2] = int(round(goal[2] - offset))
        new_points = list(planner.get_path(start=tuple(start), goal=tuple(goal)))
        new_points.pop(0)
        previous_angles_2 = previous_angles_1
        print(f"NEW POINTS: {new_points}")
        for new_point in new_points:
            new_point = list(new_point)
            new_point[0] = new_point[0] + 0.5 - pad_x_before
            new_point[1] = new_point[1] + 0.5 - pad_y_before
            new_point[2] = new_point[2] + offset
            print(f"MODIFIED POINT NEW: {new_point}")
            previous_angles_2 = move_to_point(
                direction,
                new_point,
                robot,
                num_steps,
                baseID=baseID,
                robot_id=robot_id,
                previous_angles=previous_angles_2[-1].flatten().tolist()[0],
                place_block=place_block,
                block_on_ee=block_on_ee,
            )
        # previous_angles_2 = move_to_point(direction, stop_above, robot, num_steps, baseID=baseID, robot_id=robot_id,
        #                                   previous_angles=previous_angles_1[-1].flatten().tolist()[0],
        #                                   place_block=place_block)
        move_to_point(
            direction,
            point,
            robot,
            num_steps,
            baseID=baseID,
            robot_id=robot_id,
            previous_angles=previous_angles_2[-1].flatten().tolist()[0],
            place_block=place_block,
            block_on_ee=block_on_ee,
        )

        robot.primary_ee = "D" if robot.primary_ee == "A" else "A"
        # robot.primary_ee = baseID  # Switching base to ee

    return robot, True


def get_path_to_point(
    robot,
    current_position,
    destination,
    simulator_communicator,
    blueprint,
    old_path=None,
    desired_ee="A",
):
    """
    Returns a path to the specified point

    :param robot: Robot object
    :param destination: Tuple consisting of 3d location and face to reach
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """
    logger.debug("Getting a path to return")
    armReach = [2.38, 2.38]
    # armReach = [2.2, 1.5]

    # armReach = [1.5, 1.5]

    # logger.debug(f"BLUEPRINT: {blueprint}")

    faceStarPlanner = FaceStar(blueprint, armReach)

    try:
        direction = destination[3]
    except IndexError:
        logger.warning("No destination specified, setting to default 'top'")
        direction = "top"

    point = list(destination)

    # TODO: Have conversion for other directions too
    modified_det = np.array([point[0] + 0.5, point[1] + 0.5, point[2]])
    d_pose = np.array(create_point_from_homogeneous_transform(robot.DEE_POSE))
    a_pose = np.array(create_point_from_homogeneous_transform(robot.AEE_POSE))

    # print(type(d_pose))
    if np.linalg.norm(d_pose - modified_det) <= 0.01:
        logger.debug("D link already close enough to destination, returning empty path")
        return []

    if np.linalg.norm(a_pose - modified_det) <= 0.01:
        logger.debug("A link already close enough to destination, returning empty path")
        return []

    if direction == "top" or direction == "bottom":
        point[0] = int(point[0])
        point[1] = int(point[1])
        point[2] = int(point[2]) - 1
    if direction == "left" or direction == "right":
        point[0] = point[0] + 1.37  # -1.37
        point[1] = point[1] - 0.5
        point[2] = point[2] + 0.87  # -.87
    if direction == "front" or direction == "back":
        point[0] = point[0] - 0.5
        point[1] = point[1] - 1
        point[2] = point[2] - 0.5
    logger.debug(f"Going to point: {point}")

    current_position.xPos = int(current_position.xPos)
    current_position.yPos = int(current_position.yPos)
    current_position.zPos = int(current_position.zPos) - 1

    if current_position.ee_on_face != robot.primary_ee:
        logger.exception(
            f"Start face ee ({current_position.ee_on_face}) does not match goal face ee ({robot.primary_ee})"
        )
        logger.exception(
            f"Current Position: {(current_position.xPos, current_position.yPos, current_position.zPos, current_position.ee_on_face)}"
        )
        logger.exception(
            f"Goal Position: {(point[0], point[1], point[2], direction, desired_ee)}"
        )
    try:
        logger.debug(f"Path Planner searching with desired ee: {desired_ee}")
        path = faceStarPlanner.get_path(
            start=current_position,
            goal=BlockFace(point[0], point[1], point[2], direction, desired_ee),
        )
    except Exception as e:
        logger.exception(e)
        logger.error(f"Path planner unable to find path to {point}")
        path = []

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

    def __init__(
        self,
        robot,
        key,
        robot_communicator,
        simulator_communicator,
        blueprint,
        name="RemoveBlock",
        remove_block_key="remove_block/block_to_remove",
        blueprint_key="state/blueprint",
        blocks_robot_has_moved_key="state/blocks_robot_has_moved",
        holding_blocks_key="state/blocks_being_held",
        robot_model_key="state/robot",
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
            "blocks_robot_has_moved": blocks_robot_has_moved_key,
            "holding_blocks_key": holding_blocks_key,
            "robot_model_key": robot_model_key,
        }

        self.blackboard.register_key(
            key=str(self.key), access=py_trees.common.Access.READ
        )
        self.state.register_key(key=blueprint_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(
            key=blocks_robot_has_moved_key, access=py_trees.common.Access.WRITE
        )

        self.state.register_key(
            key=holding_blocks_key, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(key=robot_model_key, access=py_trees.common.Access.WRITE)
        # self.blackboard.register_key(key=remove_block_key, access=py_trees.common.Access.READ)
        self.robot_communicator = robot_communicator
        self.simulator_communicator = simulator_communicator

    def setup(self):
        """

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

        """
        self.percentage_completion = 0
        self.block_to_remove = self.blackboard.get(str(self.key))
        self.blueprint = self.state.get(name=self.keys["blueprint"])
        # self.block = self.blackboard.get(self.keys["remove_block_key"])

        # print(self.blackboard)

    def update(self):
        """

        :return:
        """
        new_status = py_trees.common.Status.RUNNING
        # print(self.blackboard)
        # self.block_to_remove = self.blackboard.get(str(self.key))
        # self.block = self.blackboard.get(self.keys["remove_block_key"])
        # if self.parent_connection.poll():
        #     self.percentage_completion = self.parent_connection.recv().pop()
        #     if self.percentage_completion == 100:
        #         new_status = py_trees.common.Status.SUCCESS
        remove_block_action = remove_block(
            self.robot,
            self.block_to_remove,
            self.simulator_communicator,
            blueprint=self.blueprint,
            structure_communicator=self.robot_communicator,
        )
        if remove_block_action is True:
            new_status = py_trees.common.Status.SUCCESS
            # print(f"Before error {self.block_to_remove}")

            blocks_robot_has_moved = self.state.get(
                name=self.keys["blocks_robot_has_moved"]
            )
            blocks_robot_has_moved.append(
                BlockMoved(
                    id=self.block_to_remove.id,
                    location=self.block_to_remove.location,
                    placed_block=False,
                )
            )
            self.state.set(
                name=self.keys["blocks_robot_has_moved"], value=blocks_robot_has_moved
            )

            self.blueprint[self.block_to_remove.location] = 0
            self.state.set(name=self.keys["blueprint"], value=self.blueprint)
            self.blocks_moved = self.state.get(self.keys["holding_blocks_key"])

            self.robot_model = self.state.get(self.keys["robot_model_key"])
            #
            baseID = "D" if self.robot_model.primary_ee == "A" else "A"
            #
            # link_selector = 0 if baseID == "A" else 3
            # self.robot_model.links[link_selector].length *= 2
            self.blocks_moved[baseID] = self.block_to_remove.id
            self.state.set(name=self.keys["holding_blocks_key"], value=self.blocks_moved)
            # self.state.set(name=self.keys["robot_model_key"], value=self.robot_model)
        else:
            new_status = py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = (
                f"Robot has finished removing block at location {self.block_to_remove}"
            )
            self.logger.debug(
                "%s.update()[%s->%s][%s]"
                % (
                    self.__class__.__name__,
                    self.status,
                    new_status,
                    self.feedback_message,
                )
            )
        else:
            self.feedback_message = (
                f"Robot has failed to remove block at location {self.block_to_remove}"
            )
            self.logger.debug(
                "%s.update()[%s][%s]"
                % (self.__class__.__name__, self.status, self.feedback_message)
            )
        return new_status

    def terminate(self, new_status):
        """

        :param new_status:
        """
        self.logger.debug(
            "%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status)
        )


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

    def __init__(
        self,
        robot,
        key,
        robot_communicator,
        simulator_communicator,
        blueprint,
        state_key="state/block_has_been_placed",
        place_block_key="place_block/location_to_place_block",
        blocks_to_move_key="state/blocks_to_move",
        remove_block_key="remove_block/block_to_remove",
        blueprint_key="state/blueprint",
        blocks_robot_has_moved_key="state/blocks_robot_has_moved",
        name="PlaceBlock",
        holding_blocks_key="state/blocks_being_held",
        robot_model_key="state/robot",
    ):
        """
        Default construction.
        """
        super(PlaceBlock, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.robot = robot
        self.key = key
        self.robot_model_key = robot_model_key
        self.state_key = state_key
        self.blackboard = self.attach_blackboard_client("State", "place_block")
        self.blackboard.register_key(key=str(key), access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=remove_block_key, access=py_trees.common.Access.READ
        )
        self.blueprint = blueprint
        self.state = self.attach_blackboard_client()
        self.state.register_key(key=state_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(
            key=blocks_robot_has_moved_key, access=py_trees.common.Access.WRITE
        )

        self.keys = {
            "place_block_key": place_block_key,
            "robot_state": state_key,
            "remove_block_key": remove_block_key,
            "blueprint": blueprint_key,
            "blocks_to_move_key": blocks_to_move_key,
            "blocks_robot_has_moved": blocks_robot_has_moved_key,
            "holding_blocks_key": holding_blocks_key,
            "robot_model_key": robot_model_key,
        }

        # self.blackboard.register_key()

        self.state.register_key(
            key=self.keys["place_block_key"], access=py_trees.common.Access.READ
        )
        self.state.register_key(
            key=self.keys["remove_block_key"], access=py_trees.common.Access.READ
        )
        self.state.register_key(key=blueprint_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(
            key=holding_blocks_key, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(
            key=blocks_to_move_key, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(key=robot_model_key, access=py_trees.common.Access.WRITE)
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
        place_block_action = place_block(
            robot=self.robot,
            location_to_set_block=self.location_to_place_block,
            simulator_communicator=self.simulator_communicator,
            block=self.move_block,
            blueprint=self.blueprint,
            structure_communicator=self.robot_communicator,
        )

        if place_block_action is True:
            new_status = py_trees.common.Status.SUCCESS
            self.blueprint[self.location_to_place_block] = 1
            self.state.set(name=self.keys["blueprint"], value=self.blueprint)

            blocks_robot_has_moved = self.state.get(
                name=self.keys["blocks_robot_has_moved"]
            )
            blocks_robot_has_moved.append(
                BlockMoved(
                    id=self.move_block.id,
                    location=self.location_to_place_block,
                    placed_block=True,
                )
            )
            self.state.set(
                name=self.keys["blocks_robot_has_moved"], value=blocks_robot_has_moved
            )
            self.blocks_moved = self.state.get(self.keys["holding_blocks_key"])

            self.robot_model = self.state.get(self.robot_model_key)

            baseID = "D" if self.robot_model.primary_ee == "A" else "A"

            self.blocks_moved[baseID] = None
            # self.blocks_moved[self.robot_model.primary_ee] = None
            # link_selector = 0 if baseID == "A" else 3
            # self.robot_model.links[link_selector].length *= 0.5
            # self.state.set(name=self.keys["robot_model_key"], value=self.robot_model)
            logger.debug(f"Removing block id from blocks being held: {self.blocks_moved}")
            self.state.set(name=self.keys["holding_blocks_key"], value=self.blocks_moved)

            self.blocks_to_move.pop()
            self.state.set(
                name=self.keys["blocks_to_move_key"], value=self.blocks_to_move
            )
        else:
            new_status = py_trees.common.Status.FAILURE

        if new_status == py_trees.common.Status.SUCCESS:
            # TODO: Change base so it is not none (use blackboard)
            self.robot_communicator.robot_communicator.send_communication(
                topic=self.robot,
                message=StatusUpdateMessage(
                    status=RobotBehaviors.MOVE,
                    payload=PlacedBlockUpdateMessagePayload(
                        robot_base=None, block_placed=self.location_to_place_block
                    ),
                ),
            )

            self.state.set(name=self.state_key, value=True)
            self.feedback_message = f"Robot has finished placing block at location {self.location_to_place_block}"
            self.logger.debug(
                "%s.update()[%s->%s][%s]"
                % (
                    self.__class__.__name__,
                    self.status,
                    new_status,
                    self.feedback_message,
                )
            )
        else:
            self.feedback_message = f"Robot has failed to place block at location {self.location_to_place_block}"
            self.logger.debug(
                "%s.update()[%s][%s]"
                % (self.__class__.__name__, self.status, self.feedback_message)
            )
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

    def __init__(
        self,
        robot,
        key,
        robot_communicator,
        simulator_communicator,
        blueprint,
        name="NavigateToPoint",
        current_position_key="current_position",
        robot_model_key="robot",
        robot_status_key="robot_status",
        blueprint_key="blueprint",
        holding_blocks_key="blocks_being_held",
    ):
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
            "robot_status": robot_status_key,
            "holding_blocks_key": holding_blocks_key,
        }
        self.current_position_key = current_position_key
        self.robot_model_key = robot_model_key
        self.blackboard.register_key(key=self.key, access=py_trees.common.Access.READ)
        self.state.register_key(key=self.key, access=py_trees.common.Access.WRITE)
        self.state.register_key(
            key=current_position_key, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(key=robot_model_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=blueprint_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=robot_status_key, access=py_trees.common.Access.READ)
        self.state.register_key(
            key=holding_blocks_key, access=py_trees.common.Access.READ
        )

        self.blueprint = blueprint
        self.robot_communicator = robot_communicator
        self.simulator_communicator = simulator_communicator

        self.inching = False
        self.toggle_for_searching_every_other = True

    def setup(self):
        """

        """
        pass

    def initialise(self):
        """

        """
        self.reached_point = (True, None)
        self.point_to_reach = self.blackboard.get(self.key)
        self.blueprint = self.state.get(name=self.keys["blueprint"])
        self.current_position = self.state.get(self.current_position_key)

        # # TODO this is hardcoded to only convert for the TOP face case
        # self.current_position.convert_pos_from_simulator_to_robot()

        logger.info(
            f"[{self.name.upper()}]: Current Position: ({self.current_position.xPos}, "
            f"{self.current_position.yPos}, {self.current_position.zPos}, {self.current_position.face})"
        )
        logger.info(f"[{self.name.upper()}]: Got point to reach: {self.point_to_reach}")
        self.robot = self.state.get(self.robot_model_key)
        self.holding_blocks = self.state.get(self.keys["holding_blocks_key"])
        logger.exception(f"Point to reach: {self.point_to_reach}")
        basedID = "A" if self.robot.primary_ee == "D" else "D"
        if self.holding_blocks[self.robot.primary_ee] is not None:
            goal_ee = self.robot.primary_ee
        elif self.holding_blocks[basedID] is not None:
            goal_ee = basedID
        else:
            goal_ee = self.robot.primary_ee

        # if len(self.point_to_reach) == 4:

        # logger.exception(f"I am here setting the default to this")
        # goal_ee = self.point_to_reach[-1]
        # if self.inching:
        #     if self.toggle_for_searching_every_other or len(self.path) < 2:
        #         self.path = get_path_to_point(
        #             self.robot,
        #             current_position=self.current_position,
        #             destination=self.point_to_reach,
        #             simulator_communicator=self.robot_communicator,
        #             blueprint=self.blueprint,
        #             desired_ee=goal_ee
        #         )
        #         self.toggle_for_searching_every_other = False
        #     else:
        #         self.toggle_for_searching_every_other = True
        # else:
        self.path = get_path_to_point(
            self.robot,
            current_position=self.current_position,
            destination=self.point_to_reach,
            simulator_communicator=self.robot_communicator,
            blueprint=self.blueprint,
            desired_ee=goal_ee,
        )

        # print(self.blackboard)

    def update(self):
        """

        :return:
        """
        logger.debug(f"[{self.name.upper()}]: In Navigate to Point")
        new_status = py_trees.common.Status.RUNNING
        # try:
        #     self.point_to_reach = self.blackboard.get(str(self.key))
        # except KeyError:
        #     return py_trees.common.Status.FAILURE
        self.blueprint = self.state.get(name=self.keys["blueprint"])
        # self.percentage_completion = 0
        if len(self.path) > 0 and self.reached_point[0]:
            self.next_point = self.path.pop(0)

            place_block = 0
            modified_goal = np.array(self.point_to_reach[:3])
            modified_goal[2] -= 1

            robot_status = self.state.get(name=self.keys["robot_status"])
            self.holding_blocks = self.state.get(self.keys["holding_blocks_key"])
            self.block_on_ee = self.holding_blocks[self.robot.primary_ee]
            if (
                np.linalg.norm(modified_goal - np.array(self.next_point[0:3])) <= 0.1
                and (
                    robot_status == RobotBehaviors.BUILD
                    or robot_status == RobotBehaviors.FERRY
                )
                # or self.block_on_ee
            ):
                logger.exception("This is the blueprint at this point")
                logger.exception(
                    self.blueprint[modified_goal[0]][modified_goal[1]][modified_goal[2]]
                )
                logger.exception(self.blueprint)
                logger.exception(modified_goal)
                if modified_goal[0] == 0 and modified_goal[1] == 0:
                    place_block = 2
                    logger.exception("Therefore I am removing a block")
                else:
                    place_block = 1
                    logger.exception("Therefore I am placing a block")
                # logger.debug("Placing block set to true")

            self.robot, success = follow_path(
                self.robot,
                [self.next_point],
                robot_id=self.robot_id,
                place_block=place_block,
                blueprint=self.blueprint,
                block_on_ee=self.block_on_ee,
            )
            self.state.set(name=self.robot_model_key, value=self.robot)
            if success:

                base = (
                    self.robot.DEE_POSE
                    if self.robot.primary_ee == "A"
                    else self.robot.AEE_POSE
                )

                # base = create_point_from_homogeneous_transform(
                #     self.robot.base)

                baseID = "D" if self.robot.primary_ee == "A" else "A"

                base_block_face = BlockFace(base[0, 3], base[1, 3], base[2, 3], "top", baseID)

                # TODO block face is not correct
                self.state.set(name=self.current_position_key, value=base_block_face)

                # logger.debug(f"ROBOT BASE: {self.robot.base}")
                logger.debug(f"Next point moving to: {self.next_point}")
            if not success:
                base = (
                    self.robot.DEE_POSE
                    if self.robot.primary_ee == "A"
                    else self.robot.AEE_POSE
                )

                # base = create_point_from_homogeneous_transform(
                #     self.robot.base)

                baseID = "D" if self.robot.primary_ee == "A" else "A"

                base_block_face = BlockFace(base[0, 3], base[1, 3], base[2, 3], "top", baseID)

                # TODO block face is not correct
                self.state.set(name=self.current_position_key, value=base_block_face)

                # logger.debug(f"ROBOT BASE: {self.robot.base}")
                logger.debug(f"Next point moving to: {self.next_point}")
                return py_trees.common.Status.RUNNING
        if len(self.path) <= 0 and self.reached_point[0]:
            new_status = py_trees.common.Status.SUCCESS

        if new_status == py_trees.common.Status.SUCCESS:
            self.state.set(name=self.key, value=True)
            self.feedback_message = "Reached destination"
            self.logger.debug(
                "%s.update()[%s->%s][%s]"
                % (
                    self.__class__.__name__,
                    self.status,
                    new_status,
                    self.feedback_message,
                )
            )
        else:
            if self.reached_point[0]:
                self.feedback_message = f"Reached point: {self.reached_point[1]}"
            else:
                self.feedback_message = (
                    f"(Still) Moving to point ({self.reached_point[1]})"
                )
            self.logger.debug(
                "%s.update()[%s][%s]"
                % (self.__class__.__name__, self.status, self.feedback_message)
            )
        return new_status

    def terminate(self, new_status):
        """
        Nothing to clean up in this example.
        """
        self.logger.debug(
            "%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status)
        )


def create_root(
    robot, robot_communicator, simulator_communicator, blueprint, set_variables=False
):
    """
    Create the behavior tree made up of the following behaviors:

    NavigateToPoint, RemoveBlock, NavigateToPoint, PlaceBlock

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :param set_variables: Used for testing purposes, can set blackboard variables to original with
    :return:
    """

    task_one = NavigateToPoint(
        name="MoveToBlockToRemove",
        key="point_to_reach",
        robot=robot,
        robot_communicator=robot_communicator,
        simulator_communicator=simulator_communicator,
        blueprint=blueprint,
    )
    task_two = RemoveBlock(
        name="RemoveBlock",
        key="block_to_remove",
        robot=robot,
        robot_communicator=robot_communicator,
        simulator_communicator=simulator_communicator,
        blueprint=blueprint,
    )
    task_three = NavigateToPoint(
        name="MoveToPlaceBlock",
        key="point_to_reach_2",
        robot=robot,
        robot_communicator=robot_communicator,
        simulator_communicator=simulator_communicator,
        blueprint=blueprint,
    )
    task_four = PlaceBlock(
        name="PlaceBlock",
        key="location_to_place_block",
        robot=robot,
        robot_communicator=robot_communicator,
        simulator_communicator=simulator_communicator,
        blueprint=blueprint,
    )

    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off",
        tasks=[task_one, task_two, task_three, task_four],
    )

    root = py_trees.composites.Selector(name="Root")

    if set_variables:
        set_blackboard_variable = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination",
            variable_name="navigation/point_to_reach",
            variable_value=(6, 6, 6, "Left"),
        )

        set_blackboard_variable1 = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination",
            variable_name="place_block/location_to_place_block",
            variable_value=(4, 4, 4, "Left"),
        )

        set_blackboard_variable2 = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination",
            variable_name="navigation/point_to_reach_2",
            variable_value=(0, 0, 0, "Left"),
        )

        set_blackboard_variable3 = py_trees.behaviours.SetBlackboardVariable(
            name="Set destination",
            variable_name="remove_block/block_to_remove",
            variable_value=(9, 9, 9, "Left"),
        )

        # write_blackboard_variable = BlackboardWriter(name="Writer")
        check_blackboard_variable = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check set variable",
            variable_name="asdf",
            expected_value=str((6, 6, 6)),
        )
        mid = py_trees.composites.Sequence("Blackboard Demo")
        mid.add_children(
            [
                set_blackboard_variable,
                set_blackboard_variable1,
                set_blackboard_variable2,
                set_blackboard_variable3,
                check_blackboard_variable,
            ]
        )
        root.add_children([mid, piwylo])
    else:
        root.add_children([piwylo])

    return root


##############################################################################
# Main
##############################################################################


def main():
    """

    """
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


def get_motion_planning_behaviors_tree(
    robot_communicator, simulator_communicator, robot, blueprint
):
    """

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """
    return create_root(
        robot_communicator=robot_communicator,
        simulator_communicator=simulator_communicator,
        robot=robot,
        blueprint=blueprint,
    )


def get_move_to_point_tree(robot_communicator, simulator_communicator, robot, blueprint):
    """

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """

    task_one = NavigateToPoint(
        name="MoveToBlockToRemove",
        key="point_to_reach",
        robot=robot,
        robot_communicator=robot_communicator,
        simulator_communicator=simulator_communicator,
        blueprint=blueprint,
    )
    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off", tasks=[task_one]
    )

    root = py_trees.composites.Selector(name="Root")
    root.add_children([piwylo])

    return root


if __name__ == "__main__":
    main()
