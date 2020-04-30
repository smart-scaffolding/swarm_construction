import argparse
from collections import defaultdict
from random import choice

import numpy as np
import py_trees
from logzero import logger

import components.robot.config as config
from components.robot.behaviors.move_blocks import create_move_blocks_root
from components.robot.behaviors.move_robot import create_move_robot_root
from components.robot.behaviors.update_state import create_update_behavior_root
from components.robot.behaviors.wait import create__waiting_root
from components.robot.common.states import *
from components.robot.communication.communicate_with_simulator import (
    SimulatorCommunication,
)
from components.robot.communication.communicate_with_structure import (
    StructureCommunication,
)
from components.robot.communication.heartbeat import start_heartbeat
from components.robot.communication.messages import *
from components.robot.motionplanning import model, helpers
from components.robot.pathplanning.searches.face_star import BlockFace
from components.structure.behaviors.building.common_building import Block

configuration = None


class RobotMain:
    """

    """

    def __init__(self):
        self.configuration = config
        try:
            self.id = config.ROBOT_ID.encode("UTF-8")
            self.heartbeat_connection_in = config.communication["heartbeat_connection_in"]
            self.heartbeat_connection_out = config.communication[
                "heartbeat_connection_out"
            ]
            self.blueprint = config.BLUEPRINT

            if self.configuration.TESTING:
                args = command_line_argument_parser().parse_args()
                self.id = str(args.robot_id).encode("UTF-8")
                self.position = str(args.position).encode("UTF-8")
                self.position = self.position.decode().strip(" []").split(",")
                self.position = [float(x) for x in self.position]
                logger.info(f"ROBOT ID: {self.id}")
                logger.info(f"ROBOT Position: {self.position}")

            if self.configuration.SIMULATE:
                self.simulator_send_messages_socket = config.communication[
                    "simulator_send_messages_port"
                ]
            self.receive_messages_socket = config.communication["receive_messages_port"]
            self.send_messages_socket = config.communication["send_messages_port"]
        except BaseException:
            raise AttributeError("Must define all parameters in configuration file")

        self.structure_communicator = StructureCommunication(
            receive_messages_socket=self.receive_messages_socket,
            send_messages_socket=self.send_messages_socket,
            send_topics=self.id,
            receive_topics=self.id,
        )

        if self.configuration.SIMULATE:
            logger.debug("SIMULATION ENABLED")
            self.simulator_communicator = SimulatorCommunication(
                send_messages_socket=self.simulator_send_messages_socket,
                send_topics=self.id,
            )

    def initialize_communications(self):
        """

        :return:
        """
        start_heartbeat(
            id=self.id,
            connection_in=self.heartbeat_connection_in,
            connection_out=self.heartbeat_connection_out,
        )
        self.structure_communicator.initialize_communication_with_structure()
        if self.configuration.SIMULATE:
            self.simulator_communicator.initialize_communication_with_simulator()

    def create_behavior_tree(self, blueprint):
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
            blueprint=blueprint,
        )
        build_behavior = create_move_blocks_root(
            ferry=False,
            robot_communicator=communicator,
            simulator_communicator=simulator_communicator,
            robot=self.id,
            blueprint=blueprint,
        )
        wait_behavior = create__waiting_root(robot_communicator=communicator)
        move_behavior = create_move_robot_root(
            robot_communicator=communicator,
            simulator_communicator=simulator_communicator,
            robot=self.id,
            blueprint=blueprint,
        )
        update_behavior = create_update_behavior_root(
            robot_communicator=communicator, blueprint=blueprint
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
    """

    :return:
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-r", "--receive", type=int, help="Select port to receive values")
    parser.add_argument("-s", "--send", type=int, help="Select port to send values")
    parser.add_argument("-i", "--robot_id", type=str, help="Robot id")
    parser.add_argument(
        "-p",
        "--position",
        nargs="+",
        type=float,
        help="Position of robot as tuple (x, y, z)",
    )
    return parser


if __name__ == "__main__":

    header = "x SWARM CONSTRUCTION ROBOT DEMO x"
    logger.info("\n")
    logger.info("x" * len(header))
    logger.info(header)
    logger.info("x" * len(header))
    logger.info("\n")
    import time
    time.sleep(5)
    robot = RobotMain()
    root = robot.create_behavior_tree(blueprint=None)

    a_end_effector = robot.position
    # a_end_effector = [3.5, 0.5, 1]
    x, y, z = a_end_effector
    d_end_effector = [x + 2, y, z]

    blueprint = config.BLUEPRINT

    robot_model = model.Inchworm(
        blueprint=blueprint,
        a_link_starting_pos=a_end_effector,
        d_link_starting_pos=d_end_effector,
    )

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
    writer.register_key(key="state/robot", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/blueprint", access=py_trees.common.Access.WRITE)
    writer.register_key(
        key="state/blocks_being_held", access=py_trees.common.Access.WRITE
    )
    writer.register_key(
        key="state/blocks_robot_has_moved", access=py_trees.common.Access.WRITE
    )

    block = Block(final_destination=(6, 3, 1))
    block.set_next_location((3, 0, 1))
    block.location = (6, 6, 1)

    block2 = Block(final_destination=(8, 6, 1))
    block2.set_next_location((3, 1, 1))
    block2.location = (3, 6, 1)

    block3 = Block(final_destination=(8, 6, 1))
    block3.set_next_location((0, 0, 1))
    block3.location = (8, 8, 1)

    block4 = Block(final_destination=(8, 6, 1))
    block4.set_next_location((1, 1, 1))
    block4.location = (8, 0, 1)

    block5 = Block(final_destination=(8, 6, 1))
    block5.set_next_location((1, 0, 1))
    block5.location = (0, 8, 1)

    block6 = Block(final_destination=(8, 6, 1))
    block6.set_next_location((1, 0, 1))
    block6.location = (3, 2, 4)

    block7 = Block(final_destination=(8, 6, 1))
    block7.set_next_location((0, 0, 1))
    block7.location = (3, 1, 1)

    block8 = Block(final_destination=(8, 6, 1))
    block8.set_next_location((1, 0, 1))
    block8.location = (3, 2, 1)

    block9 = Block(final_destination=(8, 6, 1))
    block9.set_next_location((1, 1, 1))
    block9.location = (4, 0, 1)

    block10 = Block(final_destination=(8, 6, 1))
    block10.set_next_location((0, 1, 1))
    block10.location = (4, 1, 1)

    block11 = Block(final_destination=(8, 6, 1))
    block11.set_next_location((5, 0, 1))
    block11.location = (0, 1, 1)

    block12 = Block(final_destination=(8, 6, 1))
    block12.set_next_location((5, 0, 2))
    block12.location = (0, 1, 1)

    block13 = Block(final_destination=(8, 6, 1))
    block13.set_next_location((4, 0, 1))
    block13.location = (0, 1, 1)

    block14 = Block(final_destination=(8, 6, 1))
    block14.set_next_location((3, 0, 2))
    block14.location = (0, 1, 1)

    block15 = Block(final_destination=(8, 6, 1))
    block15.set_next_location((0, 0, 1))
    block15.location = (10, 9, 8)

    blocks = [
        # block,
        # block2,
        # block3,
        # block4,
        # block5
        # block6
        # block7,
        # block8,
        # block9,
        # block10
        # block11,
        # block12,
        # block13,
        # block14
        # block15
        # Block(location=(3, 1, 1), next_destination=(6, 4, 1), final_destination=(6, 4, 1)),
        # Block(location=(3, 2, 1), next_destination=(6, 5, 1), final_destination=(6, 5, 1)),
        # Block(location=(3, 0, 2), next_destination=(7, 3, 1), final_destination=(7, 3, 1)),
        # Block(location=(3, 1, 2), next_destination=(7, 4, 1), final_destination=(7, 4, 1)),
        # Block(location=(3, 2, 2), next_destination=(7, 5, 1), final_destination=(7, 5, 1)),
        # Block(location=(3, 0, 3), next_destination=(8, 3, 1), final_destination=(8, 3, 1)),
        # Block(location=(3, 1, 3), next_destination=(8, 4, 1), final_destination=(8, 4, 1)),
        # Block(location=(3, 2, 3), next_destination=(8, 5, 1), final_destination=(8, 5, 1)),
    ]

    destinations = [
        (2, 2, 1),
        (2, 1, 1),
        (3, 1, 1),
        (3, 2, 1),
        (3, 3, 1),
        (2, 3, 1),
        (1, 3, 1),
        (1, 2, 1),
        (1, 1, 1),
        (1, 0, 1),
        (2, 0, 1),
        (3, 0, 1),
        (4, 0, 1),
        (4, 1, 1),
        (4, 2, 1),
        (4, 3, 1),
        (4, 4, 1),
        (3, 4, 1),
        (2, 4, 1),
        (1, 4, 1),
        (0, 4, 1),
        (0, 3, 1),
        (0, 2, 1),
        (0, 1, 1),
        (0, 0, 1),
    ]

    for destination in destinations:
        block = Block(final_destination=(8, 6, 1))
        block.set_next_location(destination)
        block.location = (0, 0, 1)
        blocks.append(block)
    blocks.reverse()

    # for block in blocks:
    #     robot.simulator_communicator.send_communication(
    #         topic=block.id,
    #         message=BlockLocationMessage(
    #             block_id=block.id,
    #             location=(
    #                 block.location[0],
    #                 block.location[1] + 0.5,
    #                 block.location[2] + 0.5,
    #             ),
    #         ),
    #     )

    writer.set(name="state/blocks_to_move", value=blocks)
    # If setting to ferry/move,
    writer.set(name="state/robot_status", value=RobotBehaviors.WAIT)
    # must set block_has_been_placed to true

    # Set to true if trying to place block
    writer.set(name="state/block_has_been_placed", value=False)
    # writer.set(name="state/point_to_reach", value=False)
    # Set to false if trying to move
    writer.set(name="state/point_to_reach", value=True)
    # writer.set(name="state/location_to_move_to",
    #            value=(6, 0, 1, "top"))
    writer.set(name="state/location_to_move_to", value=(6, 6, 1, "top", "D"))
    writer.set(name="state/robot", value=robot_model)
    writer.set(name="state/blueprint", value=robot.blueprint)
    writer.set(name="state/blocks_robot_has_moved", value=[])

    writer.set(name="state/blocks_being_held", value=defaultdict(lambda: None))

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

    # writer.set(name="state/current_position", value=BlockFace(robot.position[0], robot.position[1], robot.position[2],
    #                                                           'top', 'D'))
    writer.set(
        name="state/current_position",
        value=BlockFace(
            a_end_effector[0], a_end_effector[1], a_end_effector[2], "top", "A"
        ),
    )

    behaviour_tree.setup(timeout=15)

    robot.initialize_communications()
    helpers.send_to_simulator(
        base=robot_model.AEE_POSE,

        trajectory=np.array([0, 0, 62, -1.23793284e02, -28, 0]),
        id=robot.id,
    )
    helpers.send_to_simulator(
        base=robot_model.AEE_POSE,
        trajectory=np.array([0, 0, 62, -1.23793284e02, -28, 0]),
        id=robot.id,
    )
    helpers.send_to_simulator(
        base=robot_model.AEE_POSE,
        trajectory=np.array([0, 0, 62, -1.23793284e02, -28, 0]),
        id=robot.id,
    )

    def generate_results():
        """

        """
        print("I have been called")
        import matplotlib.pyplot as plt
        from collections import Counter

        # Pie chart, where the slices will be ordered and plotted counter-clockwise:
        blocks_robot_has_moved = writer.get(name="state/blocks_robot_has_moved")
        block_count = Counter(blocks_robot_has_moved)
        fig = plt.figure()
        ax = fig.add_axes([0, 0, 1, 1])
        ax.bar(list(block_count.keys()), list(block_count.values()))
        plt.show()
        logger.info(f"Number of times touched blocks: {sum(block_count.values())}")
        logger.info(f"Blocks: {block_count.items()}")

        plt.figure()
        labels = "Frogs", "Hogs", "Dogs", "Logs"
        sizes = [15, 30, 45, 10]
        explode = (0, 0, 0, 0)  # only "explode" the 2nd slice (i.e. 'Hogs')

        fig1, ax1 = plt.subplots()
        ax1.pie(
            sizes,
            explode=explode,
            labels=labels,
            autopct="%1.1f%%",
            shadow=True,
            startangle=90,
        )
        ax1.axis("equal")  # Equal aspect ratio ensures that pie is drawn as a circle.

        plt.show()
        exit()

    # ####################
    # # Tick Tock
    # ####################

    while True:
        # signal(SIGINT, generate_results)
        # signal(SIGTERM, generate_results)

        try:
            behaviour_tree.tick()
        except KeyboardInterrupt:
            print("I have received a keyboard interrupt")
            # generate_results()
            break
        except KeyError as e:
            logger.exception(f"Key error exception caught in main: {e}")
            raise e
            # continue
