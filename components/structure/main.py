from components.structure.communication import RobotCommunication
from components.structure.communication import SimulatorCommunication
from components.structure.communication.heartbeater import start_hearbeat_detector
from components.structure.communication.messages import *
from components.robot.communication.messages import StatusUpdateMessagePayload as RobotStatusUpdateMessagePayload
from components.structure.common.common import create_point_from_homogeneous_transform
import components.structure.config as config
from components.simulator.model.graphics import vtk_named_colors
from components.structure.behaviors.building.assign_robots_min_distance import Robot, assign_robots_closest_point, Block
from queue import PriorityQueue, Empty
from functools import total_ordering
from components.structure.behaviors.building.merge_paths import Node
from random import sample, choice
import time
# from queue import Queue
# import asyncio
from multiprocessing import Queue
import numpy as np

configuration = None


class StructureMain:
    def __init__(self):
        self.configuration = config
        self.robot_queue = Queue()
        self.known_robots = {}
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
            self.receive_messages_socket = config.communication["receive_messages_port"]
            self.send_messages_socket = config.communication["send_messages_port"]
        except:
            raise Exception("Must define all parameters in configuration file")

        self.robot_communicator = RobotCommunication(receive_messages_socket=self.receive_messages_socket,
                                                             send_messages_socket=self.send_messages_socket,
                                                             send_topics=self.known_robots,
                                                             receive_topics=self.known_robots)

        if config.SIMULATE:
            self.simulator_send_messages_socket = config.communication["simulator_send_messages_port"]
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
        if config.SIMULATE:
            self.initialize_simulator(blueprint=blueprint, colors=colors)

    def initialize_simulator(self, blueprint, colors):
        simulator_message = SimulatorStructureMessage(blueprint=blueprint, colors=colors)
        self.simulator_communicator.initialize_communication_with_simulator()
        # self.simulator_communicator.send_communication(message=simulator_message, topic=b"STRUCTURE")



if __name__ == '__main__':

    structure = StructureMain()

    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])

    bx, by, bz = blueprint.shape
    colors = [[[vtk_named_colors(["DarkGreen"])] * bz] * by] * bx

    # colors[0][0][0] = vtk_named_colors(["Blue"])
    # colors[0]4][1] = vtk_named_colors(["Blue"])
    # colors[0][4][4] = vtk_named_colors(["Blue"])
    # colors[0][4][7] = vtk_named_colors(["Blue"])

    # blueprint = np.array([
    #                          [[1] * 9] * 9,
    #                      ] * 9)
    #
    # colors = np.array([[['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen']],
    #
    #    [['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen']],
    #
    #    [['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen']],
    #
    #    [['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['DarkGreen', 'DarkGreen', 'DarkGreen', 'DarkGreen', 'Yellow',
    #      'Yellow', 'Yellow', 'Yellow', 'Yellow'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen'],
    #     ['Red', 'Red', 'Red', 'Red', 'DarkGreen', 'DarkGreen',
    #      'DarkGreen', 'DarkGreen', 'DarkGreen']],
    #
    #    [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange']],
    #
    #    [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange']],
    #
    #    [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange']],
    #
    #    [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange']],
    #
    #    [['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Orange', 'Orange', 'Orange', 'Orange', 'Blue', 'Blue', 'Blue',
    #      'Blue', 'Blue'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange'],
    #     ['Yellow', 'Yellow', 'Yellow', 'Yellow', 'Orange', 'Orange',
    #      'Orange', 'Orange', 'Orange']]])

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



    # while True:
    #     print("Ticking")
    #     time.sleep(0.5)
    #     if not structure.robot_queue.empty():
    #         new_robot = structure.robot_queue.get()
    #
    #         new_robot_obj = Robot(id=new_robot, pos=None)
    #
    #         structure.known_robots[new_robot] = new_robot_obj
    #
    #         print(f"New robot added to storage: {new_robot_obj}")
    #     if len(structure.known_robots) > 0:
    #         random_robot = sample(list(structure.known_robots.values()), 1)[0]
    #         print(f"Random robot: {random_robot}")
    #
    #         # print(f"[Structure] sending random robot a message -> HI {random_robot}")
    #         structure.robot_communicator.send_communication(topic=random_robot.id, message=MoveToPointMessage(
    #             destination=choice([(1.5, 1.5, 1, "Top"), (4.5, 1.5, 1, "Top"), (7.5, 1.5, 1, "Top"),
    #                                  (1.5, 4.5, 1, "Top"), (4.5, 4.5, 1, "Top"), (5.5, 4.5, 1, "Top"),
    #                                  (1.5, 7.5, 1, "Top"), (3.5, 7.5, 1, "Top"), (5.5, 7.5, 1, "Top"),
    #                                  ])))
    #         messages = structure.robot_communicator.get_communication()
    #         for update in messages:
    #             topic, message = update
    #             print(f"[Structure] got message(s) from robot {topic}-> {messages}")
    #             print(message.robot_status)
    #             print(message.payload)
    #
    #             if isinstance(message.payload, RobotStatusUpdateMessagePayload):
    #                 structure.known_robots[topic].pos = create_point_from_homogeneous_transform(
    #                     message.payload.robot_base)
    #                 print(create_point_from_homogeneous_transform(message.payload.robot_base))
    #
    #             structure.known_robots[topic].update_status(message.robot_status)

    path1 = [Node(wavefront_order=2, id=1, child=2, direction="RIGHT", pos=(1.5, 1.5)),
             Node(3, 2, 5, "UP", pos=(4.5, 1.5)),
             Node(4, 5, 8, "UP", pos=(4.5, 4.5)),
             Node(5, 8, None, None, pos=(4.5, 7.5))]
    path2 = [Node(2, 1, 2, "RIGHT", pos=(1.5, 1.5)), Node(3, 2, 5, "UP", pos=(4.5, 1.5)), Node(4, 5, 6, "RIGHT",
                                                                                               pos=(4.5, 4.5)),
             Node(5, 6, None, None, pos=(7.5, 4.5))]

    all_nodes = {}

    # for point in path
    merged_path = path1
    q = PriorityQueue()

    # MERGE = False
    for node in path2:
        if node in path1:
            # MERGE = True

            index_in_path = path1.index(node)
            node_in_path = path1[index_in_path]

            node_in_path.direction = node_in_path.direction.union(node.direction)
            node_in_path.children = node_in_path.children.union(node.children)
            node_in_path.num_blocks += node.num_blocks

        else:
            all_nodes[node.id] = (node, None)
            q.put(node)

    goals = [Node(5, 8, None, None, pos=(7.5, 1.5)), Node(5, 6, None, None, pos=(7.5, 4.5))]

    goals.sort(key=lambda x: x.order)
    goal_ids = [8, 6]
    # print(goals)


    blocks_to_move = {8: [Block(location=(0, 0, 1), final_destination=(6, 0, 1)),
                          Block(location=(0, 1, 1), final_destination=(6, 1, 1)),
                          Block(location=(0, 2, 1), final_destination=(6, 2, 1)),
                          Block(location=(1, 0, 1), final_destination=(7, 0, 1)),
                          Block(location=(1, 1, 1), final_destination=(7, 1, 1)),
                          Block(location=(1, 2, 1), final_destination=(7, 2, 1)),
                          Block(location=(2, 0, 1), final_destination=(8, 0, 1)),
                          Block(location=(2, 1, 1), final_destination=(8, 1, 1)),
                          Block(location=(2, 2, 1), final_destination=(8, 2, 1)),
                          ],
                      6: [Block(location=(0, 0, 2), final_destination=(6, 3, 1)),
                          Block(location=(0, 1, 2), final_destination=(6, 4, 1)),
                          Block(location=(0, 2, 2), final_destination=(6, 5, 1)),
                          Block(location=(1, 0, 2), final_destination=(7, 3, 1)),
                          Block(location=(1, 1, 2), final_destination=(7, 4, 1)),
                          Block(location=(1, 2, 2), final_destination=(7, 5, 1)),
                          Block(location=(2, 0, 2), final_destination=(8, 3, 1)),
                          Block(location=(2, 1, 2), final_destination=(8, 4, 1)),
                          Block(location=(2, 2, 2), final_destination=(8, 5, 1)),
                          ],
                     }


    for node in path1:
        all_nodes[node.id] = (node, None)
        q.put(node)

    currently_claimed_set = []  # TODO: Ensure this data structure cannot be modified, important to preserve order

    robots = [
        Robot(id=b'ROBOT_1', pos=(1.5, 1.5), claimed_division=1),
        Robot(id=b'ROBOT_2', pos=(4.5, 1.5), claimed_division=2),
        # Robot(id=b'ROBOT_3', pos=(4.5, 4.5), claimed_division=3),
        # Robot(id=4, pos=(0, 2), claimed_division=3),
        # Robot(id=3, pos=(1, 1), claimed_division=5),
        # Robot(id=5, pos=(2, 2), claimed_division=9),
    ]

    # print(f"All nodes values: {list(all_nodes.values())}")
    # assign_robots_closest_point(robots, all_nodes.values()[0])

    for bot in range(len(robots)):
        if q.empty():
            break
        else:
            node = q.get()
            currently_claimed_set.append(node)

    # print(currently_claimed_set)

    assign_robots_closest_point(robots, currently_claimed_set, structure.robot_communicator)  # assign robots
    for bot in robots:  # update dictionary to include robot with claimed division
        node, _ = all_nodes[bot.target.id]
        all_nodes[bot.target.id] = (node, bot)

    currently_working = []
    node1 = currently_claimed_set[0]
    currently_working.append(node1)

    ferry_blocks = []
    while len(currently_working) > 0:
        # print(repr(node), end="\n\n")
        # TODO: TELL FIRST ROBOT TO START DOING WORK

        for node in currently_working:
            for blocks in ferry_blocks:
                blocks.assigned_node = node.id

            time.sleep(5)
            node, robot = all_nodes[node.id]
            # if node.id in goal_ids:
            #     structure.robot_communicator.send_communication(topic=robot.id, message=BuildMessage(
            #         blocks_to_move=blocks_to_move[node.id]))
            #
            # else:
            #     flattened = [val for sublist in list(blocks_to_move.values()) for val in sublist]
            #
            #     ferry_blocks_id = list(filter(lambda x: x.assigned_node == node.id, flattened))
            #     ferry_blocks = ferry_blocks_id[0:9]
            #
            #     structure.robot_communicator.send_communication(topic=robot.id, message=FerryBlocksMessage(
            #         blocks_to_move=ferry_blocks))
            #
            #     print(f"Sending robot {robot.id} message to ferry blocks: {ferry_blocks}")
            # print(f"Node in currently_working: {node}")
            # print(f"\nDOING WORK on node: {node.id}")
            # print(f"\nDOING WORK on node: {node.id}")
            # print(f"\nDOING WORK on node: {node.id}")
            # print(f"\nDOING WORK on node: {node.id}")
            #
            # time.sleep(5)


            # Robot has finished working
            try:
                print("Getting new node")
                new_node = q.get(timeout=2)
                print(f"New node: {new_node}")
                currently_claimed_set.append(new_node)
                # currently_claimed_set.remove(node)
                print("Done updating")
            except(Empty):
                print("Queue is empty, continuing")
                # continue

            print("\nAssigning robots to points")

            # TODO: Change assign_robots to deal with if num of points less than number of robots
            if len(currently_claimed_set) > len(robots):
                currently_claimed_set.remove(node)
            #
            # print(f"Currently Claimed Set: {len(currently_claimed_set)} Number of robots: {len(robots)}")
            # print(f"Current claimed set: {currently_claimed_set}")
            assign_robots_closest_point(robots, currently_claimed_set, structure.robot_communicator)
            time.sleep(5)
            print("Updating dictionary with new robot positions")
            for bot in robots:  # update dictionary to include robot with claimed division
                update_node, _ = all_nodes[bot.claimed_division.id]
                all_nodes[bot.claimed_division.id] = (update_node, bot)
            print("\n\n")

            # for node in currently_claimed_set:
            for child in node.children:
                if child is None:
                    print("No children to notify, continuing")
                    continue
                else:
                    notify_node, robot = all_nodes[child]  #
                    print(f"Notifiying child node with id: {child} {robot.id}")
                    currently_working.append(notify_node)
                    # currently_claimed_set.remove
            currently_working.remove(node)
