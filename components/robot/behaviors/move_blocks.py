##############################################################################
# Imports
##############################################################################

import py_trees
from .motion_planning_behaviors import get_motion_planning_behaviors_tree
from components.robot.common.states import Block, MoveBlocksStore, Division, RobotBehaviors
from components.robot.communication.messages import StatusUpdateMessage, BlockLocationMessage, FerryBlocksStatusFinished
import time


##############################################################################
# Classes
##############################################################################


class MoveBlocks(py_trees.behaviour.Behaviour):
    def __init__(self, name, status_identifier, robot_communicator,
                 simulator_communicator, navigation_first_key="navigation/point_to_reach",
                 place_block_key="place_block/location_to_place_block",
                 navigation_second_key="navigation/point_to_reach_2",
                 remove_block_key="remove_block/block_to_remove",
                 blocks_to_move_key="blocks_to_move",
                 robot_state="robot_status",
                 block_placed_state="block_has_been_placed",
                 ):
        super().__init__(name=name)
        self.communicator = robot_communicator.robot_communicator
        self.simulator_communicator = simulator_communicator.robot_communicator
        self.robot_id = robot_communicator.robot_id
        self.blackboard = self.attach_blackboard_client()
        self.state = self.attach_blackboard_client("State", "state")
        self.status_identifier = status_identifier
        self.keys = {
            "place_block_key": place_block_key,
            "navigation_first_key": navigation_first_key,
            "navigation_second_key": navigation_second_key,
            "remove_block_key": remove_block_key,
            "blocks_to_move_key": blocks_to_move_key,
            "robot_state": robot_state,
            "block_placed_state": block_placed_state,
        }
        self.blackboard.register_key(key=navigation_first_key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=place_block_key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=navigation_second_key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=remove_block_key, access=py_trees.common.Access.WRITE)

        self.state.register_key(key=blocks_to_move_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=robot_state, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=block_placed_state, access=py_trees.common.Access.WRITE)

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        blocks_to_move_key = self.keys["blocks_to_move_key"]
        self.blocks_to_move = self.state.get(blocks_to_move_key)

        # print(self.blocks_to_move)
        robot_status_key = self.keys["robot_state"]
        self.robot_status = self.state.get(robot_status_key)

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if self.robot_status != self.status_identifier:
            # print(f"[{self.name.upper()}]: Returning success {self.robot_status} {self.status_identifier}")
            return py_trees.common.Status.SUCCESS

        if len(self.blocks_to_move) <= 0:

            print(f"[{self.name.upper()}]: Success! All blocks have been moved")
            response_message = StatusUpdateMessage(status=self.status_identifier, payload="All blocks have been moved "
                                                                                         "successfully")
            self.communicator.send_communication(topic=self.robot_id, message=response_message)

            self.communicator.send_communication(topic=self.robot_id, message=StatusUpdateMessage(
                status=RobotBehaviors.WAIT, payload=FerryBlocksStatusFinished()))

            self.communicator.send_communication(topic=self.robot_id, message=StatusUpdateMessage(
                status=RobotBehaviors.WAIT, payload=FerryBlocksStatusFinished()))

            self.communicator.send_communication(topic=self.robot_id, message=StatusUpdateMessage(
                status=RobotBehaviors.WAIT, payload=FerryBlocksStatusFinished()))

            self.communicator.send_communication(topic=self.robot_id, message=StatusUpdateMessage(
                status=RobotBehaviors.WAIT, payload=FerryBlocksStatusFinished()))


            print("Sent finished status to structure (done moving blocks)")
            self.state.set(name=self.keys["robot_state"], value=RobotBehaviors.WAIT)
            return py_trees.common.Status.SUCCESS
        if self.state.get(name=self.keys["block_placed_state"]) is False:
            # print("SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS HAVE BEEN PLACED")
            # print(f"BLOCK: {self.move_block.id} {self.move_block.location}")
            # self.simulator_communicator.send_communication(topic=self.move_block.id, message=BlockLocationMessage(
            #     block_id=self.move_block.id, location=self.move_block.next_destination))

            response_message = StatusUpdateMessage(status=self.status_identifier, payload="Moving to place block")
            self.communicator.send_communication(topic=self.robot_id, message=response_message)
            print(f"[{self.name.upper()}]: Still moving")
            return py_trees.common.Status.RUNNING
        else:
            response_message = StatusUpdateMessage(status=self.status_identifier, payload="Block placed, getting next "
                                                                                         "block to place")
            self.communicator.send_communication(topic=self.robot_id, message=response_message)

            print(f"[{self.name.upper()}]: Moving blocks...")
            self.state.set(name=self.keys["block_placed_state"], value=False)
            self.move_block = self.blocks_to_move.pop()
            print(f"[{self.name.upper()}]: New block being moved: {self.move_block}")
            self.block_destination = self.move_block.location
            print(f"[{self.name.upper()}]: New block destination: {self.block_destination}")
            print("\n")
            self.blackboard.set(name=self.keys["remove_block_key"], value=self.move_block)
            self.blackboard.set(name=self.keys["navigation_first_key"], value=self.block_destination) #TODO: Change
            # to previous location of block
            self.blackboard.set(name=self.keys["navigation_second_key"], value=self.move_block.next_destination) #TODO: Change
            # to next destination of block
            self.blackboard.set(name=self.keys["place_block_key"], value=self.move_block.next_destination)

            # self.simulator_communicator.send_communication(topic=self.move_block.id, message=BlockLocationMessage(
            #     block_id=self.move_block.id, location=self.move_block.next_destination))
            # print("SENDING COMMUNICATION TO SIMULATOR THAT BLOCKS HAVE BEEN PLACED")
            # print(f"BLOCK: {self.move_block.id} {self.move_block.location}")
            # self.simulator_communicator.send_communication(topic=self.move_block.id, message=BlockLocationMessage(
            #     block_id=self.move_block.id, location=self.move_block.next_destination))
        return new_status


class FerryBlocks(MoveBlocks):
    pass


class Build(MoveBlocks):
    pass


def create_move_blocks_root(robot_communicator, robot, blueprint, simulator_communicator=None, ferry=True):
    if ferry:
        move_action = py_trees.decorators.RunningIsFailure(child=FerryBlocks(name="Ferry",
                                                                             status_identifier=RobotBehaviors.FERRY,
                                                                             robot_communicator=robot_communicator,
                                                                             simulator_communicator=simulator_communicator))
    else:
        move_action = py_trees.decorators.RunningIsFailure(child=MoveBlocks(name="Build",
                                                                            status_identifier=RobotBehaviors.BUILD,
                                                                            robot_communicator=robot_communicator,
                                                                            simulator_communicator=simulator_communicator))

    motion_planning_behaviors = get_motion_planning_behaviors_tree(robot_communicator=robot_communicator,
                                                                   simulator_communicator=simulator_communicator,
                                                                   robot=robot, blueprint=blueprint)
    root = py_trees.composites.Selector(name="Root")

    move = py_trees.composites.Selector(name="Move Blocks Motion Planning")
    move.add_children([move_action, motion_planning_behaviors])
    root.add_children([move])
    return root

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the demo script.
    """

    py_trees.logging.level = py_trees.logging.Level.INFO

    root = create_move_blocks_root(ferry=True)


    behaviour_tree = py_trees.trees.BehaviourTree(root)

    blocks_to_place = [Block(position=(0, 0, 0, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
                       Block(position=(1, 1, 1, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
                       Block(position=(2, 2, 2, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top"))]

    division = Division()

    move_store = MoveBlocksStore(blocks_to_remove=blocks_to_place, division=division)


    writer = py_trees.blackboard.Client(name="Writer")
    writer.register_key(key="state/blocks_to_move", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/block_has_been_placed", access=py_trees.common.Access.WRITE)

    writer.set(name="state/blocks_to_move", value=move_store)
    writer.set(name="state/robot_status", value=RobotBehaviors.FERRY)
    writer.set(name="state/block_has_been_placed", value=True)


    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    for unused_i in range(1, 25):
        try:
            behaviour_tree.tick()
            print("Tree is ticking")
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

if __name__ == '__main__':
    main()