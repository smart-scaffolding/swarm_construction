from robot.communication.heartbeat import start_heartbeat
from robot.communication.communicate_with_simulator import SimulatorCommunication
from robot.communication.communicate_with_structure import StructureCommunication
from robot.behaviors.move_blocks import create_move_blocks_root
from robot.behaviors.wait import create__waiting_root
from robot.behaviors.move_robot import create_move_robot_root
from robot.behaviors.update_state import create_update_behavior_root
from robot.common.states import *
import robot.config as Config
import py_trees
import py_trees.console as console
from random import randint
import time
import json
import argparse

configuration = None


class RobotMain:
    def __init__(self):
        self.configuration = Config
        try:
            self.id = Config.ROBOT_ID.encode('UTF-8')
            self.heartbeat_connection_in = Config.communication["heartbeat_connection_in"]
            self.heartbeat_connection_out = Config.communication["heartbeat_connection_out"]
            if Config.TESTING:
                args = command_line_argument_parser().parse_args()
                self.id = str(args.robot_id).encode('UTF-8')
                print(self.id)
            if Config.SIMULATE:
                self.simulator_send_messages_socket = Config.communication["simulator_send_messages_port"]
            self.receive_messages_socket = Config.communication["receive_messages_port"]
            self.send_messages_socket = Config.communication["send_messages_port"]
        except:
            raise Exception("Must define all parameters in configuration file")

        self.structure_communicator = StructureCommunication(receive_messages_socket=self.receive_messages_socket,
                                                             send_messages_socket=self.send_messages_socket,
                                                             send_topics=self.id,
                                                             receive_topics=self.id)

        if Config.SIMULATE:

            self.simulator_communicator = SimulatorCommunication(
                send_messages_socket=self.simulator_send_messages_socket,
                send_topics=b"SIMULATOR")

    def initialize_communications(self):
        """

        :return:
        """
        print(self.id)
        start_heartbeat(id=self.id, connection_in=self.heartbeat_connection_in,
                        connection_out=self.heartbeat_connection_out)
        self.structure_communicator.initialize_communication_with_structure()
        if Config.SIMULATE:
            self.simulator_communicator.initialize_communication_with_simulator()


    def create_behavior_tree(self):
        """

        :return:
        """

        behaviors = py_trees.composites.Sequence(name="Behaviors")

        communicator = RobotCommunicator(robot_communicator=self.structure_communicator, robot_id=self.id)
        simulator_communicator = RobotCommunicator(robot_communicator=self.simulator_communicator, robot_id=self.id)

        ferry_behavior = create_move_blocks_root(ferry=True, robot_communicator=communicator,
                                                 simulator_communicator=simulator_communicator)
        build_behavior = create_move_blocks_root(ferry=False, robot_communicator=communicator,
                                                 simulator_communicator=simulator_communicator)
        wait_behavior = create__waiting_root(robot_communicator=communicator)
        move_behavior = create_move_robot_root(robot_communicator=communicator,
                                               simulator_communicator=simulator_communicator)
        update_behavior = create_update_behavior_root(robot_communicator=communicator)

        behaviors.add_children([move_behavior, ferry_behavior, build_behavior, wait_behavior])

        # root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
        #     children=[update_behavior]))
        root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())


        root.add_children([update_behavior, behaviors])
        return root

def command_line_argument_parser():
    parser = argparse.ArgumentParser(
                                        formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('-r', '--receive', type=int, help='Select port to receive values')
    parser.add_argument('-s', '--send', type=int, help='Select port to send values')
    parser.add_argument('-i', '--robot_id', type=str, help='Robot id')
    return parser


if __name__ == '__main__':
    from robot.communication.messages import *

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
    writer.register_key(key="state/block_has_been_placed", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/blocks_to_move", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/location_to_move_to", access=py_trees.common.Access.WRITE)
    writer.register_key(key="state/point_to_reach", access=py_trees.common.Access.WRITE)
    #
    writer.set(name="state/blocks_to_move", value=None)
    writer.set(name="state/robot_status", value=RobotBehaviors.WAIT)
    writer.set(name="state/block_has_been_placed", value=True)
    writer.set(name="state/point_to_reach", value=False)
    writer.set(name="state/location_to_move_to", value=(7, 7, 7, "Top"))


    behaviour_tree.setup(timeout=15)
    #
    # ####################
    # # Tick Tock
    # ####################


    while True:
    # for unused_i in range(1, 50):
        try:
            behaviour_tree.tick()
            print("Tree is ticking")

            time.sleep(1)
        except KeyboardInterrupt:
            break
        except KeyError as e:
            print(e)
            continue
    # print("\n")
    # while True:
    #     print("Ticking")
    #     time.sleep(0.5)
    #     robot.structure_communicator.send_communication(topic=robot.id, message="Hello from robot")