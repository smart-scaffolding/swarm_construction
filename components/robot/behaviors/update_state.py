##############################################################################
# Imports
##############################################################################

import py_trees
import time
from components.robot.common.states import RobotBehaviors

##############################################################################
# Classes
##############################################################################


class UpdateState(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        status_identifier,
        robot_communicator,
        navigation_first_key="navigation/point_to_reach",
        place_block_key="place_block/location_to_place_block",
        navigation_second_key="navigation/point_to_reach_2",
        remove_block_key="remove_block/block_to_remove",
        blocks_to_move_key="blocks_to_move",
        robot_state="robot_status",
        block_placed_state="block_has_been_placed",
        location_to_move_to_key="location_to_move_to",
        destination_reached="point_to_reach",
        behavior_state="behavior_state",
        behavior_timer=None,
    ):
        super().__init__(name=name)
        self.state = self.attach_blackboard_client("State", "state")
        self.blackboard = self.attach_blackboard_client()

        self.status_identifier = status_identifier

        self.keys = {
            "place_block_key": place_block_key,
            "navigation_first_key": navigation_first_key,
            "navigation_second_key": navigation_second_key,
            "remove_block_key": remove_block_key,
            "blocks_to_move_key": blocks_to_move_key,
            "robot_state": robot_state,
            "block_placed_state": block_placed_state,
            "location_to_move_to_key": location_to_move_to_key,
            "destination_reached": destination_reached,
            "behavior_state": behavior_state,
        }
        self.blackboard = self.attach_blackboard_client()

        self.blackboard.register_key(
            key=navigation_first_key, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=place_block_key, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=navigation_second_key, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=remove_block_key, access=py_trees.common.Access.WRITE
        )

        self.state.register_key(
            key=blocks_to_move_key, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(key=robot_state, access=py_trees.common.Access.WRITE)
        self.state.register_key(
            key=block_placed_state, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(
            key=destination_reached, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(
            key=location_to_move_to_key, access=py_trees.common.Access.WRITE
        )
        self.state.register_key(key=behavior_state, access=py_trees.common.Access.WRITE)

        self.communicator = robot_communicator.robot_communicator
        self.robot_id = robot_communicator.robot_id

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.behavior_timer = behavior_timer

    def initialise(self):
        # robot_status_key = self.keys["robot_state"]
        # self.robot_status = self.state.get(robot_status_key)
        pass

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        updates = self.communicator.get_communication()
        for update in updates:
            # print(f"[{self.name.upper()}]: update -> {update}")
            topic, message = update
            message_id = message.message_id
            if message_id == RobotBehaviors.FERRY:
                # print(
                #     f"[{self.name.upper()}]: Received ferry message {message}, sending to handler"
                # )
                self.handle_ferry_message(message)

            elif message_id == RobotBehaviors.BUILD:
                # print(
                #     f"[{self.name.upper()}]: Received build message {message}, sending to handler"
                # )
                self.handle_build_message(message)

            elif message_id == RobotBehaviors.WAIT:
                # print(
                #     f"[{self.name.upper()}]: Received wait message {message}, sending to handler"
                # )
                self.handle_wait_message(message)

            elif message_id == RobotBehaviors.MOVE:
                # print(
                #     f"[{self.name.upper()}]: Received move message {message}, sending to handler"
                # )
                self.handle_move_message(message)

            else:
                raise Exception("Unknown message type received")

        new_status = py_trees.common.Status.SUCCESS
        # print(f"[{self.name.upper()}]: Updates received")
        if self.behavior_timer:
            self.behavior_timer.update_time(self.name)
        self.state.set(name=self.keys["behavior_state"], value=self.name)
        return new_status

    def handle_ferry_message(self, update):
        self.state.set(name=self.keys["blocks_to_move_key"], value=update.blocks_to_move)
        self.state.set(name=self.keys["robot_state"], value=update.message_id)
        self.state.set(name=self.keys["block_placed_state"], value=True)

    def handle_build_message(self, update):
        self.state.set(name=self.keys["blocks_to_move_key"], value=update.blocks_to_move)
        self.state.set(name=self.keys["robot_state"], value=update.message_id)
        self.state.set(name=self.keys["block_placed_state"], value=True)

    def handle_wait_message(self, update):
        self.state.set(name=self.keys["robot_state"], value=update.message_id)

    def handle_move_message(self, update):
        self.state.set(name=self.keys["robot_state"], value=update.message_id)
        self.state.set(name=self.keys["block_placed_state"], value=True)
        self.state.set(
            name=self.keys["location_to_move_to_key"], value=update.location_to_move_to
        )
        self.state.set(name=self.keys["destination_reached"], value=False)


def create_update_behavior_root(robot_communicator, behavior_timer=None):
    # wait_action = py_trees.decorators.RunningIsFailure(child=Wait(name="Wait", status_identifier="WAIT"))

    update_state = UpdateState(
        name="Update",
        status_identifier=RobotBehaviors.UPDATE,
        robot_communicator=robot_communicator,
        behavior_timer=behavior_timer,
    )

    root = py_trees.composites.Selector(name="Root")

    root.add_children([update_state])
    return root


##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the demo script.
    """

    py_trees.logging.level = py_trees.logging.Level.INFO

    root = create_update_behavior_root()

    ####################
    # Execute
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(root)

    writer = py_trees.blackboard.Client(name="Writer")
    writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    writer.set(name="state/robot_status", value=RobotBehaviors.WAIT)

    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    for unused_i in range(1, 25):
        try:
            behaviour_tree.tick()
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()
