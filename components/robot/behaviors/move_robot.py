##############################################################################
# Imports
##############################################################################

import py_trees
from .motion_planning_behaviors import get_move_to_point_tree
from components.robot.common.states import Block, MoveBlocksStore, Division, RobotBehaviors
from components.robot.communication.messages import StatusUpdateMessage
import time


##############################################################################
# Classes
##############################################################################



class MoveToNewLocation(py_trees.behaviour.Behaviour):
    def __init__(self, name, status_identifier, robot_communicator, navigation_first_key="navigation/point_to_reach",
                 location_to_move_to_key="location_to_move_to",
                 robot_state="robot_status",
                 block_placed_state="block_has_been_placed",
                 destination_reached="point_to_reach"
                 ):
        super().__init__(name=name)
        self.communicator = robot_communicator.robot_communicator
        self.robot_id = robot_communicator.robot_id

        self.blackboard = self.attach_blackboard_client()
        self.state = self.attach_blackboard_client("State", "state")
        self.status_identifier = status_identifier
        self.keys = {
            "navigation_first_key": navigation_first_key,
            "location_to_move_to_key": location_to_move_to_key,
            "robot_state": robot_state,
            "block_placed_state": block_placed_state,
            "destination_reached": destination_reached,
        }
        self.blackboard.register_key(key=navigation_first_key, access=py_trees.common.Access.WRITE)

        self.state.register_key(key=location_to_move_to_key, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=robot_state, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=block_placed_state, access=py_trees.common.Access.WRITE)
        self.state.register_key(key=destination_reached, access=py_trees.common.Access.WRITE)

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):

        # print(self.blocks_to_move)
        robot_status_key = self.keys["robot_state"]
        self.robot_status = self.state.get(robot_status_key)

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        location_to_move_to_key = self.keys["location_to_move_to_key"]
        self.destination = self.state.get(location_to_move_to_key)

        if self.robot_status != self.status_identifier:
            print(f"[{self.name.upper()}]: Returning success {self.robot_status} {self.status_identifier}")
            return py_trees.common.Status.SUCCESS

        elif self.state.get(name=self.keys["destination_reached"]) is True:
            print(f"[{self.name.upper()}]: Returning success {self.robot_status} {self.status_identifier}")
            response_message = StatusUpdateMessage(status=self.status_identifier, payload="Destination has been reached")
            self.communicator.send_communication(topic=self.robot_id, message=response_message)
            self.state.set(name=self.keys["robot_state"], value=RobotBehaviors.WAIT)
            return py_trees.common.Status.SUCCESS
        else:
            print(f"[{self.name.upper()}]: Moving to location...")
            response_message = StatusUpdateMessage(status=self.status_identifier, payload="Moving to location...")
            self.communicator.send_communication(topic=self.robot_id, message=response_message)

            self.blackboard.set(name=self.keys["navigation_first_key"], value=self.destination)
        return new_status

def create_move_robot_root(robot_communicator, simulator_communicator, robot):
    # move_action = py_trees.decorators.RunningIsFailure(child=FerryBlocks(name="Ferry", status_identifier="FERRY"))
    move_action = py_trees.decorators.RunningIsFailure(child=MoveToNewLocation(name="Move",
                                                                               status_identifier=RobotBehaviors.MOVE,
                                                                               robot_communicator=robot_communicator))

    move_to_point_behavior = get_move_to_point_tree(robot_communicator=robot_communicator,
                                                    simulator_communicator=simulator_communicator, robot=robot)
    root = py_trees.composites.Selector(name="Root")

    move = py_trees.composites.Selector(name="Move to point")
    move.add_children([move_action, move_to_point_behavior])
    root.add_children([move])
    return root

##############################################################################
# Main
##############################################################################


def main():

    # args = command_line_argument_parser().parse_args()
    # print(description())
    py_trees.logging.level = py_trees.logging.Level.INFO

    root = create_move_robot_root()


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
    writer.register_key(key="state/location_to_move_to", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/point_to_reach", access=py_trees.common.Access.WRITE)

    writer.set(name="state/blocks_to_move", value=move_store)
    writer.set(name="state/robot_status", value=RobotBehaviors.MOVE)
    writer.set(name="state/block_has_been_placed", value=True)
    writer.set(name="state/point_to_reach", value=False)
    writer.set(name="state/location_to_move_to", value=(7, 7, 7, "Top"))


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