from components.robot.communication.heartbeat import start_heartbeat

# from .communication import SimulatorCommunication
from components.robot.communication import *

# from components.robot.communication import StructureCommunication
from components.robot.behaviors.move_blocks import create_move_blocks_root
from components.robot import *
from components.robot.behaviors.move_robot import create_move_robot_root
from components.robot.behaviors.update_state import create_update_behavior_root
from components.robot.behaviors.wait import create__waiting_root
from components.robot.common.states import *
import components.robot.config as config
from components.robot.communication.communicate_with_simulator import (
    SimulatorCommunication,
)
from components.robot.communication.communicate_with_structure import (
    StructureCommunication,
)
from components.robot.communication.messages import *
from components.robot.pathplanning.searches.face_star import BlockFace
from components.structure.behaviors.building.common_building import Block
from components.robot.common.timing import BehaviorTiming

import py_trees
import time
import argparse
import numpy as np
from random import choice

configuration = None


class RobotMain:
    def __init__(self):
        self.configuration = config
        try:
            self.id = config.ROBOT_ID.encode("UTF-8")
            self.heartbeat_connection_in = config.communication["heartbeat_connection_in"]
            self.heartbeat_connection_out = config.communication[
                "heartbeat_connection_out"
            ]
            if self.configuration.TESTING:
                args = command_line_argument_parser().parse_args()
                self.id = str(args.robot_id).encode("UTF-8")
                self.configuration.ROBOT_ID = self.id
                print(self.id)
            if self.configuration.SIMULATE:
                self.simulator_send_messages_socket = config.communication[
                    "simulator_send_messages_port"
                ]
            if self.configuration.RECORD_METRICS:
                if self.configuration.TESTING:
                    self.behavior_timer = BehaviorTiming(
                        behaviors=[
                            "Wait",
                            "Update",
                            "Build",
                            "Move",
                            "Ferry",
                        ],
                        filename=f"./results/behavior_time_{self.id.decode()}_{config.EXPERIMENT_NAME}.csv",
                    )
                else:
                    self.behavior_timer = BehaviorTiming(
                        behaviors=[
                            "Wait",
                            "Update",
                            "Build",
                            "Move",
                            "Ferry",
                            ],
                        filename=self.configuration.RECORD_BEHAVIOR_TIME_FILE,
                    )
            self.receive_messages_socket = config.communication["receive_messages_port"]
            self.send_messages_socket = config.communication["send_messages_port"]
        except:
            raise Exception("Must define all parameters in configuration file")

        self.structure_communicator = StructureCommunication(
            receive_messages_socket=self.receive_messages_socket,
            send_messages_socket=self.send_messages_socket,
            send_topics=self.id,
            receive_topics=self.id,
        )

        if self.configuration.SIMULATE:
            print("SIMULATING")
            self.simulator_communicator = SimulatorCommunication(
                send_messages_socket=self.simulator_send_messages_socket,
                send_topics=self.id,
            )

    def initialize_communications(self):
        """

        :return:
        """
        print(self.id)
        start_heartbeat(
            id=self.id,
            connection_in=self.heartbeat_connection_in,
            connection_out=self.heartbeat_connection_out,
        )
        self.structure_communicator.initialize_communication_with_structure()
        if self.configuration.SIMULATE:
            self.simulator_communicator.initialize_communication_with_simulator()

    def create_behavior_tree(self):
        """

        :return:
        """

        behaviors = py_trees.composites.Sequence(name="Behaviors")

        communicator = RobotCommunicator(
            robot_communicator=self.structure_communicator, robot_id=self.id
        )
        simulator_communicator = RobotCommunicator(
            robot_communicator=self.simulator_communicator, robot_id=self.id
        )

        ferry_behavior = create_move_blocks_root(
            ferry=True,
            robot_communicator=communicator,
            simulator_communicator=simulator_communicator,
            robot=self.id,
            behavior_timer=self.behavior_timer,
        )
        build_behavior = create_move_blocks_root(
            ferry=False,
            robot_communicator=communicator,
            simulator_communicator=simulator_communicator,
            robot=self.id,
            behavior_timer=self.behavior_timer,
        )
        wait_behavior = create__waiting_root(
            robot_communicator=communicator, behavior_timer=self.behavior_timer
        )
        move_behavior = create_move_robot_root(
            robot_communicator=communicator,
            simulator_communicator=simulator_communicator,
            robot=self.id,
            behavior_timer=self.behavior_timer,
        )
        update_behavior = create_update_behavior_root(
            robot_communicator=communicator, behavior_timer=self.behavior_timer
        )

        behaviors.add_children(
            [move_behavior, ferry_behavior, build_behavior, wait_behavior]
        )

        # root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
        #     children=[update_behavior]))
        root = py_trees.composites.Parallel(
            name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )

        root.add_children([update_behavior, behaviors])
        return root


def command_line_argument_parser():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-r", "--receive", type=int, help="Select port to receive values")
    parser.add_argument("-s", "--send", type=int, help="Select port to send values")
    parser.add_argument("-i", "--robot_id", type=str, help="Robot id")
    return parser


if __name__ == "__main__":

    robot = RobotMain()
    robot.initialize_communications()
    root = robot.create_behavior_tree()

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    #
    # blocks_to_place = [Block(position=(0, 0, 0, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
    #                    Block(position=(1, 1, 1, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
    #                    Block(position=(2, 2, 2, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top"))]
    #
    # division = Division()
    #
    # move_store = FerryBlocksStore(blocks_to_remove=blocks_to_place, division=division)
    #
    writer = py_trees.blackboard.Client(name="Writer")
    # writer.register_key(key="state/blocks_to_move", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    writer.register_key(
        key="state/block_has_been_placed", access=py_trees.common.Access.WRITE
    )
    writer.register_key(key="state/blocks_to_move", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    writer.register_key(
        key="state/location_to_move_to", access=py_trees.common.Access.WRITE
    )
    writer.register_key(key="state/point_to_reach", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/current_position", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/behavior_state", access=py_trees.common.Access.WRITE)
    # blocks = [Block(location=(3, 0, 1), next_destination=(6, 3, 1), final_destination=(6, 3, 1)),
    #         Block(location=(3, 1, 1), next_destination=(6, 4, 1), final_destination=(6, 4, 1)),
    #         Block(location=(3, 2, 1), next_destination=(6, 5, 1), final_destination=(6, 5, 1)),
    #         Block(location=(3, 0, 2), next_destination=(7, 3, 1), final_destination=(7, 3, 1)),
    #         Block(location=(3, 1, 2), next_destination=(7, 4, 1), final_destination=(7, 4, 1)),
    #         Block(location=(3, 2, 2), next_destination=(7, 5, 1), final_destination=(7, 5, 1)),
    #         Block(location=(3, 0, 3), next_destination=(8, 3, 1), final_destination=(8, 3, 1)),
    #         Block(location=(3, 1, 3), next_destination=(8, 4, 1), final_destination=(8, 4, 1)),
    #         Block(location=(3, 2, 3), next_destination=(8, 5, 1), final_destination=(8, 5, 1)),
    #         ]
    # blocks.reverse()
    #
    writer.set(name="state/blocks_to_move", value=None)
    writer.set(name="state/robot_status", value=RobotBehaviors.WAIT)
    writer.set(name="state/block_has_been_placed", value=False)
    writer.set(name="state/point_to_reach", value=True)
    writer.set(name="state/location_to_move_to", value=(10, 0, 0, "top"))
    writer.set(name="state/behavior_state", value="Wait")

    choice(
        [
            (1, 1, 0, "top"),
            (4, 1, 0, "top"),
            (7, 1, 0, "top"),
            (1, 4, 0, "top"),
            (4, 4, 0, "top"),
            (5, 4, 0, "top"),
            (1, 7, 0, "top"),
            (3, 7, 0, "top"),
            (5, 7, 0, "top"),
        ]
    )

    writer.set(name="state/current_position", value=BlockFace(0, 0, 0, "top"))

    behaviour_tree.setup(timeout=15)
    #
    # ####################
    # # Tick Tock
    # ####################

    robot.behavior_timer.start_behavior_timing()
    while True:
        # for unused_i in range(1, 50):
        try:
            behaviour_tree.tick()
            # print("Tree is ticking")
            #
            # base1 = np.matrix([[1, 0, 0, 1.5],
            #                    [0, 1, 0, 0.5],
            #                    [0, 0, 1, 1.],
            #                    [0, 0, 0, 1]])
            # base2 = np.matrix([[1, 0, 0, 1.5],
            #                    [0, 1, 0, 1.5],
            #                    [0, 0, 1, 1.],
            #                    [0, 0, 0, 1]])
            # base3 = np.matrix([[1, 0, 0, 4.5],
            #                    [0, 1, 0, 0.5],
            #                    [0, 0, 1, 1.],
            #                    [0, 0, 0, 1]])
            # base4 = np.matrix([[1, 0, 0, 4.5],
            #                    [0, 1, 0, 1.5],
            #                    [0, 0, 1, 3.],
            #                    [0, 0, 0, 1]])
            # base5 = np.matrix([[1, 0, 0, 5.5],
            #                    [0, 1, 0, 0.5],
            #                    [0, 0, 1, 1.],
            #                    [0, 0, 0, 1]])
            #
            # base = choice([base1, base2, base3, base4, base5])
            # robot.simulator_communicator.send_communication(topic=robot.id, message=AnimationUpdateMessage(
            #     robot_base=base))
            # time.sleep(1)
            # writer.set(name="state/robot_status", value=RobotBehaviors.MOVE)
            # writer.set(name="state/block_has_been_placed", value=True)
            # writer.set(name="state/point_to_reach", value=False)
            # writer.set(name="state/location_to_move_to",
            #            value=choice([(1.5, 1.5, 1, "Top"), (4.5, 1.5, 1, "Top"), (7.5, 1.5, 1, "Top"),
            #                          (1.5, 4.5, 1, "Top"), (4.5, 4.5, 1, "Top"), (5.5, 4.5, 1, "Top"),
            #                          (1.5, 7.5, 1, "Top"), (3.5, 7.5, 1, "Top"), (5.5, 7.5, 1, "Top"),
            #                          ]))
        except KeyboardInterrupt:
            break
        except KeyError as e:
            print(f"Key error exception caught in main: {e}")
            # raise e
            continue
    # print("\n")
    # while True:
    #     print("Ticking")
    #     time.sleep(0.5)
    # robot.structure_communicator.send_communication(topic=robot.id, message="Hello from robot")
