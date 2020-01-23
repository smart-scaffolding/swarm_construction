from zmq.eventloop import ioloop

import structure.config as Config
from structure.communication.communicate_with_robots import RobotCommunication
from structure.communication.communicate_with_simulator import SimulatorCommunication, vtk_named_colors
from structure.communication.heartbeater import start_hearbeat_detector
from structure.communication.messages import SimulatorStructureMessage
from structure.common.states import *
import py_trees
import py_trees.console as console
from random import sample
import time
import json
import argparse
# from queue import Queue
# import asyncio
from multiprocessing import Queue
import numpy as np

configuration = None


class StructureMain:
    def __init__(self):
        self.configuration = Config
        self.robot_queue = Queue()
        self.known_robots = set()
        try:
            # self.id = Config.ROBOT_ID.encode('UTF-8')
            # self.heartbeat_connection_in = Config.communication["heartbeat_connection_in"]
            # self.heartbeat_connection_out = Config.communication["heartbeat_connection_out"]
            # if Config.TESTING:
            #     args = command_line_argument_parser().parse_args()
            #     self.id = str(args.robot_id).encode('UTF-8')
            #     print(self.id)
            #     # self.receive_messages_socket = "tcp://127.0.0.1:" + str(args.receive)
            #     # self.send_messages_socket = "tcp://127.0.0.1:" + str(args.send)
            #     self.receive_messages_socket = Config.communication["receive_messages_port"]
            #     self.send_messages_socket = Config.communication["send_messages_port"]
            # else:
            self.receive_messages_socket = Config.communication["receive_messages_port"]
            self.send_messages_socket = Config.communication["send_messages_port"]
        except:
            raise Exception("Must define all parameters in configuration file")

        self.robot_communicator = RobotCommunication(receive_messages_socket=self.receive_messages_socket,
                                                             send_messages_socket=self.send_messages_socket,
                                                             send_topics=self.known_robots,
                                                             receive_topics=self.known_robots)

        if Config.SIMULATE:
            self.simulator_send_messages_socket = Config.communication["simulator_send_messages_port"]
            self.simulator_communicator = SimulatorCommunication(
                send_messages_socket=self.simulator_send_messages_socket,
                send_topics=b"STRUCTURE")

    def initialize_communications(self, blueprint=None, colors=None):
        """

        :return:
        """
        print(f"[Structure] Starting all communications")
        start_hearbeat_detector(self.robot_queue)
        self.robot_communicator.initialize_communication_with_structure()
        if Config.SIMULATE:
            self.initialize_simulator(blueprint=blueprint, colors=colors)

    def initialize_simulator(self, blueprint, colors):
        simulator_message = SimulatorStructureMessage(blueprint=blueprint, colors=colors)
        self.simulator_communicator.initialize_communication_with_simulator()
        self.simulator_communicator.send_communication(message=simulator_message, topic=b"STRUCTURE")



if __name__ == '__main__':
    from robot.communication.messages import *

    structure = StructureMain()

    # blueprint = np.array([
    #     [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    #     [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    #     [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
    #     [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
    #     [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
    #     [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    # ])
    #
    # bx, by, bz = blueprint.shape
    # colors = [[[vtk_named_colors(["DarkGreen"])] * bz] * by] * bx

    blueprint = np.array([
                             [[1] * 9] * 9,
                         ] * 9)

    colors = np.array([[['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen']],

       [['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen']],

       [['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen']],

       [['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
         'Yellow', 'Yellow', 'Yellow', 'Yellow'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen'],
        ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
         'DarkGreen', 'DarkGreen', 'DarkGreen']],

       [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange']],

       [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange']],

       [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange']],

       [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange']],

       [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
         'Blue', 'Blue'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange'],
        ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
         'Orange', 'Orange', 'Orange']]])

    structure.initialize_communications(blueprint=blueprint, colors=colors)
    # root = robot.create_behavior_tree()
    ####################
    # Rendering
    ####################
    # if args.render:
    # py_trees.display.render_dot_tree(root, with_blackboard_variables=False)
    #     sys.exit()
    # if args.render_with_blackboard_variables:
    #     py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
    #     sys.exit()

    ####################
    # Execute
    ####################
    # behaviour_tree = py_trees.trees.BehaviourTree(root)
    #
    # blocks_to_place = [Block(position=(0, 0, 0, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
    #                    Block(position=(1, 1, 1, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top")),
    #                    Block(position=(2, 2, 2, "Top"), status="Ferry", final_destination=(9, 9, 9, "Top"))]
    #
    # division = Division()
    #
    # move_store = FerryBlocksStore(blocks_to_remove=blocks_to_place, division=division)
    #
    # writer = py_trees.blackboard.Client(name="Writer")
    # writer.register_key(key="state/blocks_to_move", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/block_has_been_placed", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/blocks_to_move", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/location_to_move_to", access=py_trees.common.Access.WRITE)
    # writer.register_key(key="state/point_to_reach", access=py_trees.common.Access.WRITE)
    # #
    # writer.set(name="state/blocks_to_move", value=None)
    # writer.set(name="state/robot_status", value=RobotBehaviors.WAIT)
    # writer.set(name="state/block_has_been_placed", value=True)
    # writer.set(name="state/point_to_reach", value=False)
    # writer.set(name="state/location_to_move_to", value=(7, 7, 7, "Top"))
    #
    # # behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    # # behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    # # snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    # # behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    # # behaviour_tree.visitors.append(snapshot_visitor)
    # behaviour_tree.setup(timeout=15)
    #
    # ####################
    # # Tick Tock
    # ####################
    #
    # # py_trees.console.read_single_keypress()
    # while True:
    # # for unused_i in range(1, 50):
    #     try:
    #         behaviour_tree.tick()
    #         print("Tree is ticking")
    #         # if args.interactive:
    #         #     py_trees.console.read_single_keypress()
    #         # else:
    #         time.sleep(1)
    #     except KeyboardInterrupt:
    #         break
    #     except KeyError as e:
    #         print(e)
    #         continue
    # print("\n")



    while True:
        print("Ticking")
        time.sleep(0.5)
        if not structure.robot_queue.empty():
            new_robot = structure.robot_queue.get()
            structure.known_robots.add(new_robot)
            print(f"New robot added to storage: {new_robot}")
        if len(structure.known_robots) > 0:
            random_robot = sample(structure.known_robots, 1)[0]
            # print(f"[Structure] sending random robot a message -> HI {random_robot}")
            # structure.robot_communicator.send_communication(topic=random_robot, message=WaitMessage())
            messages = structure.robot_communicator.get_communication()
            for update in messages:
                topic, message = update
                print(f"[Structure] got message(s) from robot {topic}-> {messages}")
                print(message.robot_status)
                print(message.payload)
