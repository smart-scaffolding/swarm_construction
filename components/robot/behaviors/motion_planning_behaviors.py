
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
import numpy as np
from random import choice

##############################################################################
# Classes
##############################################################################
def remove_block(robot, block_to_pick_up, simulator_communicator):
    """
    Action used to remove a block from the structure at the specified location

    :param robot: Robot object
    :param block_to_pick_up: Block object
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """

    try:
        print(f"Robot is removing block at location {block_to_pick_up}")
        if config.SIMULATE:
            simulator_communicator.robot_communicator.send_communication("Removing block")

            print("SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS ARE BEING REMOVED")
            print(f"BLOCK: {block_to_pick_up} {block_to_pick_up.location}")
            simulator_communicator.robot_communicator.send_communication(topic=block_to_pick_up.id,
                                                                         message=BlockLocationMessage(
                block_id=block_to_pick_up.id, location=block_to_pick_up.location))

        time.sleep(0.05)
        print(f"Robot has removed up block {block_to_pick_up}")
        time.sleep(0.05)
        return True
    except KeyboardInterrupt:
        pass

def place_block(robot, simulator_communicator, location_to_set_block, block):
    """
    Action used to place a block on the structure at the specified location

    :param robot: Robot object
    :param simulator_communicator: Object to send messages to simulator
    :param location_to_set_block: Tuple denoting location and orientation to place block
    :return:
    """
    try:
        print(f"Robot is placing block at location {location_to_set_block}")
        time.sleep(0.1)
        if config.SIMULATE:
            simulator_communicator.robot_communicator.send_communication(message=
                                                                         PlacedBlockUpdateMessagePayload(
                                                                             robot_base=None,
                                                                             block_placed=location_to_set_block),
                                                                        ) #TODO:

            print("SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS HAVE BEEN PLACED")
            print(f"BLOCK: {block} {block.location}")
            simulator_communicator.robot_communicator.send_communication(topic=block.id, message=BlockLocationMessage(
                block_id=block.id, location=block.next_destination))
            # Change base not to none
        print(f"Robot has placed block at: {location_to_set_block}")
        time.sleep(0.1)
        return True
    except KeyboardInterrupt:
        pass

def ik_solver(pipe_connection, robot, simulator_communicator, percentage_done=0):
    """
    Action used to solve for inverse kinematics

    :param pipe_connection: Process connection to receive messages from parent process
    :param robot: Robot object
    :param simulator_communicator: Object to send messages to simulator
    :param percentage_done: Percentage of ik solved
    :return:
    """

    idle = True
    percentage_complete = percentage_done
    try:
        while(True):
            if pipe_connection.poll():
                pipe_connection.recv()
                percentage_complete = 0
                idle = False
            if not idle:
                percentage_complete += 10
                pipe_connection.send([percentage_complete])
                if percentage_complete == 100:
                    idle = True
                    # break
            if config.SIMULATE:
                simulator_communicator.robot_communicator.send_communication("Moving")
            # print(f"IK solver: completion done {percentage_complete}")
            time.sleep(0.01)
        return True
    except KeyboardInterrupt:
        pass



def get_path_to_point(robot, destination, simulator_communicator, old_path=None):
    """
    Returns a path to the specified point

    :param robot: Robot object
    :param destination: Tuple consisting of 3d location and face to reach
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """
    print("Getting a path to return")
    time.sleep(0.05)
    print("Path found")
    # if config.SIMULATE:
    #     simulator_communicator.robot_communicator.send_communication(message="Got path")



    # return [choice([(1, 0, 0, "Top"), (2, 0, 0, "Top"), (3, 0, 0, "Top"), (4, 0, 0, "Top")])]
    return []

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

    def __init__(self, robot, key, robot_communicator, simulator_communicator, name="RemoveBlock",
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
        self.key = key

        # self.keys = {
        #     "remove_block_key": remove_block_key,
        # }

        self.blackboard.register_key(key=str(self.key), access=py_trees.common.Access.READ)
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
        remove_block_action = remove_block(self.robot, self.block_to_remove, self.simulator_communicator)
        new_status = py_trees.common.Status.SUCCESS if remove_block_action is True else \
            py_trees.common.Status.FAILURE
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

    def __init__(self, robot, key, robot_communicator, simulator_communicator,
                 state_key="state/block_has_been_placed",
                 place_block_key="place_block/location_to_place_block",
                 remove_block_key="remove_block/block_to_remove",
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

        self.state = self.attach_blackboard_client()
        self.state.register_key(key=state_key, access=py_trees.common.Access.WRITE)
        self.keys = {
            "place_block_key": place_block_key,
            "robot_state": state_key,
            "remove_block_key": remove_block_key,
        }

        # self.blackboard.register_key()

        self.state.register_key(key=self.keys["place_block_key"], access=py_trees.common.Access.READ)
        self.state.register_key(key=self.keys["remove_block_key"], access=py_trees.common.Access.READ)
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
                                         simulator_communicator=self.simulator_communicator, block=self.move_block)
        new_status = py_trees.common.Status.SUCCESS if place_block_action is True else \
            py_trees.common.Status.FAILURE
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

    def __init__(self, robot, key, robot_communicator, simulator_communicator, name="NavigateToPoint"):
        """

        :param robot:
        :param key:
        :param robot_communicator:
        :param simulator_communicator:
        :param name:
        """
        super(NavigateToPoint, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.robot = robot
        # self.point_to_reach = destination
        self.blackboard = self.attach_blackboard_client("Navigation", "navigation")
        self.state = self.attach_blackboard_client("State", "state")
        self.key = key
        self.blackboard.register_key(key=self.key, access=py_trees.common.Access.READ)
        self.state.register_key(key=self.key, access=py_trees.common.Access.WRITE)

        self.robot_communicator = robot_communicator
        self.simulator_communicator = simulator_communicator

    def setup(self):
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.ik_planner = multiprocessing.Process(target=ik_solver, args=(self.child_connection, self.robot,
                                                                          self.simulator_communicator))
        atexit.register(self.ik_planner.terminate)
        self.ik_planner.start()

    def initialise(self):
        self.reached_point = (True, None)
        self.point_to_reach = self.blackboard.get(self.key)
        print(f"[{self.name.upper()}]: Got point to reach: {self.point_to_reach}")
        self.path = get_path_to_point(self.robot, self.point_to_reach, self.robot_communicator, self.point_to_reach)
        # print(self.blackboard)

    def update(self):
        print(f"[{self.name.upper()}]: In Navigate to Point")
        new_status = py_trees.common.Status.RUNNING
        # try:
        #     self.point_to_reach = self.blackboard.get(str(self.key))
        # except KeyError:
        #     return py_trees.common.Status.FAILURE

        self.percentage_completion = 0
        if self.path and self.reached_point[0]:
            self.next_point = self.path.pop()
            self.parent_connection.send(self.next_point)
            base = create_homogeneous_transform_from_point(np.array(self.next_point[0:3]))
            self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
                                                                              message=AnimationUpdateMessage(
                                                                                  robot_base=base))
            self.robot_communicator.robot_communicator.send_communication(topic=self.robot,
                                                                          message=StatusUpdateMessage(
                                                                              status=RobotBehaviors.MOVE,
                                                                              payload=StatusUpdateMessagePayload(
                                                                                robot_base=base)))
            print(f"Next point moving to: {self.next_point}")
        if self.parent_connection.poll():
            self.percentage_completion = self.parent_connection.recv().pop()
            if self.percentage_completion == "FAILURE":
                print(f"[{self.name.upper()}]: IK solver unable to find solution")
                return py_trees.common.Status.FAILURE
            while self.percentage_completion != 100:
                """
                Note will block other behaviors until done moving, this is to ensure
                that while robot detaches foot and moves it to new location, it can
                never be interrupted. Otherwise, robot could be stopped with foot hanging
                in midair.
                """

                base = create_homogeneous_transform_from_point(np.array(self.next_point[0:3]))
                self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
                                                                     message=AnimationUpdateMessage(
                    robot_base=base))

                self.robot_communicator.robot_communicator.send_communication(topic=self.robot,
                                                                              message=StatusUpdateMessage(
                                                                                  status=RobotBehaviors.MOVE,
                                                                                  payload=StatusUpdateMessagePayload(
                                                                                      robot_base=base)))
                self.percentage_completion = self.parent_connection.recv().pop()
                self.reached_point = (False, self.next_point)
            if self.percentage_completion == 100:
                print(f"[{self.name.upper()}]: Reached point: {self.next_point}")
                self.reached_point = (True, self.next_point)
        else:
            self.reached_point = (True, self.reached_point[1]) #TODO: Set to false, need to wait for all points to
            # finish
            point = np.array(self.point_to_reach[0:3])
            try:
                z = point[2]
            except IndexError:
                z = 2

            #TODO: Fix last point
            base = create_homogeneous_transform_from_point((point[0]+0.5, point[1]+0.5, z))
            self.simulator_communicator.robot_communicator.send_communication(topic=self.robot,
                                                                              message=AnimationUpdateMessage(
                                                                                  robot_base=base))
        if len(self.path) == 0 and self.reached_point[0]:
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

def create_root(robot, robot_communicator, simulator_communicator, set_variables=False):
    """
    Create the behavior tree made up of the following behaviors:

    NavigateToPoint, RemoveBlock, NavigateToPoint, PlaceBlock

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :param set_variables: Used for testing purposes, can set blackboard variables to test with
    :return:
    """

    task_one = NavigateToPoint(name="MoveToBlockToRemove", key="point_to_reach", robot=robot,
                               robot_communicator=robot_communicator, simulator_communicator=simulator_communicator)
    task_two = RemoveBlock(name="RemoveBlock", key="block_to_remove", robot=robot,
                           robot_communicator=robot_communicator, simulator_communicator=simulator_communicator)
    task_three = NavigateToPoint(name="MoveToPlaceBlock", key="point_to_reach_2", robot=robot,
                                 robot_communicator=robot_communicator, simulator_communicator=simulator_communicator)
    task_four = PlaceBlock(name="PlaceBlock", key="location_to_place_block", robot=robot,
                           robot_communicator=robot_communicator, simulator_communicator=simulator_communicator)

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

def get_motion_planning_behaviors_tree(robot_communicator, simulator_communicator, robot):
    """

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """
    return create_root(robot_communicator=robot_communicator, simulator_communicator=simulator_communicator, robot=robot)

def get_move_to_point_tree(robot_communicator, simulator_communicator, robot):
    """

    :param robot_communicator: Object to send messages to structure
    :param simulator_communicator: Object to send messages to simulator
    :return:
    """

    task_one = NavigateToPoint(name="MoveToBlockToRemove", key="point_to_reach", robot=robot,
                               robot_communicator=robot_communicator, simulator_communicator=simulator_communicator)
    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off",
        tasks=[task_one]
    )

    root = py_trees.composites.Selector(name="Root")
    root.add_children([piwylo])

    return root

if __name__ == '__main__':
    main()