import time
from copy import deepcopy
from multiprocessing import Queue
from queue import PriorityQueue, Empty

from logzero import logger

import components.structure.config as config
from components.robot.communication.messages import (
    FerryBlocksStatusFinished,
    MovingFinished,
)
from components.structure.behaviors.building.assign_robots_min_distance import (
    assign_robots_closest_point,
)
from components.structure.behaviors.building.common_building import (
    spiral_sort_helper,
    Block,
    Robot,
)
from components.structure.behaviors.building.select_ferry_regions import (
    determine_ferry_regions,
)
from components.structure.behaviors.divide_structure import BuildingPlanner
from components.structure.communication.communicate_with_robots import RobotCommunication
from swarm_c_library.blueprint_factory import BluePrintFactory
from components.structure.communication.communicate_with_simulator import (
    SimulatorCommunication,
)
from components.structure.communication.heartbeater import start_hearbeat_detector
from components.structure.communication.messages import *
from components.robot.communication.messages import BlockLocationMessage

configuration = None


class StructureMain:
    def __init__(
        self,
        template,
        blueprint,
        division_size=5,
        ferry_region_size=3,
        feeding_location=(0, 0),
    ):
        self.configuration = config
        # self.manager = Manager()
        self.robot_queue = Queue()
        self.known_robots = {}
        self.template = template
        self.buildingPlanner = BuildingPlanner(
            template, feeding_location=feeding_location
        )
        self.blueprint = blueprint

        self.goals = set()
        self.goal_ids = set()
        self.blocks_to_move = []
        self.all_nodes = {}
        self.nodes_to_visit = PriorityQueue()
        self.currently_claimed_set = (
            []
        )  # TODO: Ensure this data structure cannot be modified, important to preserve
        self.currently_working = []
        self.already_visited_ids = []
        self.division_size = division_size
        self.ferry_region_size = ferry_region_size

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
            config_exception = Exception(
                "Must define all parameters in configuration file"
            )
            logger.exception(config_exception)
            raise config_exception

        self.robot_communicator = RobotCommunication(
            receive_messages_socket=self.receive_messages_socket,
            send_messages_socket=self.send_messages_socket,
            send_topics=self.known_robots,
            receive_topics=self.known_robots,
        )

        if config.SIMULATE:
            self.simulator_send_messages_socket = config.communication[
                "simulator_send_messages_port"
            ]
            self.simulator_communicator = SimulatorCommunication(
                send_messages_socket=self.simulator_send_messages_socket,
                send_topics=b"STRUCTURE",
            )

    def reset_building_planner(self, template, division_size=5, feeding_location=(0, 0)):
        self.template = template
        self.buildingPlanner = BuildingPlanner(
            template, feeding_location=feeding_location
        )
        self.division_size = division_size
        # self.ferry_region_size =

    def initialize_communications(self, colors=None):
        """

        :return:
        """
        logger.info(f"[Structure] Starting all communications")
        start_hearbeat_detector(self.robot_queue)
        self.robot_communicator.initialize_communication_with_structure()
        if config.SIMULATE:
            self.initialize_simulator(colors=colors)

    def initialize_simulator(self, colors):
        # simulator_message = SimulatorStructureMessage(
        #     blueprint=self.blueprint, colors=colors
        # )
        self.simulator_communicator.initialize_communication_with_simulator()
        # self.simulator_communicator.send_communication(message=simulator_message, topic=b"STRUCTURE")

    def print_structure_status(self):
        print("\n\n\n")
        print("-" * 30)
        print("Structure Status:")
        print(f"\nGoals: {self.goals}")
        print(f"\nBlocks: {self.blocks_to_move}")
        print(f"\nCurrently Working: {self.currently_working}")
        print(f"\nCurrently Claimed: {self.currently_claimed_set}")

        print("\n\n")

    def merge_paths(self):
        values = list(self.divisions.values())
        values.sort(key=lambda x: x.order)

        self.item1 += 1
        self.item2 += 1

        skip_path1 = False
        if values[self.item1].id in self.already_visited_ids:
            skip_path1 = True
            path1 = None
            # self.item1 = None
        else:
            path1 = values[self.item1].path_to_node
        path2 = values[self.item2].path_to_node

        # for point in path

        merge = False
        # MERGE = False
        if path1 is not None:

            for node in path2:
                # merge = False
                for other_node in path1:
                    merge = False
                    if node.id == other_node.id:
                        # if node in path1:
                        # MERGE = True

                        # index_in_path = path1.index(node)
                        # node_in_path = path1[index_in_path]
                        node_in_path = other_node

                        if None in node_in_path.direction or None in node.direction:
                            self.all_nodes[node.id] = (node, None)
                            self.nodes_to_visit.put(node)
                            merge = True
                            break
                        node_in_path.direction = node_in_path.direction.union(
                            node.direction
                        )
                        node_in_path.children = node_in_path.children.union(node.children)
                        # for direction in node_in_path.num_blocks:
                        #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
                        node_in_path.num_blocks += node.num_blocks
                        merge = True
                        break
                # if node in self.need_to_visit_again:
                # MERGE = True
                # if not merge:
                #     for other_node in self.need_to_visit_again:
                #         if node.id == other_node.id:
                #             # index_in_path = self.need_to_visit_again.index(node)
                #             # node_in_path = self.need_to_visit_again[index_in_path]
                #             node_in_path = other_node
                #
                #
                #             if None in node_in_path.direction or None in node.direction:
                #                 self.all_nodes[node.id] = (node, None)
                #                 self.nodes_to_visit.put(node)
                #                 merge = True
                #                 continue
                #             node_in_path.direction = node_in_path.direction.union(node.direction)
                #             node_in_path.children = node_in_path.children.union(node.children)
                #             # for direction in node_in_path.num_blocks:
                #             #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
                #             node_in_path.num_blocks += node.num_blocks

                if not merge:
                    # else:
                    #     for other_node in self.need_to_visit_again:
                    #         if node.id == other_node.id:
                    #             # if node in path1:
                    #             # MERGE = True
                    #
                    #             # index_in_path = path1.index(node)
                    #             # node_in_path = path1[index_in_path]
                    #             node_in_path = other_node
                    #
                    #             if None in node_in_path.direction or None in node.direction:
                    #                 self.all_nodes[node.id] = (node, None)
                    #                 self.nodes_to_visit.put(node)
                    #                 merge = True
                    #                 continue
                    #             node_in_path.direction = node_in_path.direction.union(node.direction)
                    #             node_in_path.children = node_in_path.children.union(node.children)
                    #             # for direction in node_in_path.num_blocks:
                    #             #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
                    #             node_in_path.num_blocks += node.num_blocks
                    #         else:
                    self.all_nodes[node.id] = (node, None)
                    self.nodes_to_visit.put(node)
                    merge = False
        else:
            path1 = path2

        if not skip_path1:
            self.goals.add(values[self.item1])
        self.goals.add(values[self.item2])

        # self.goals.sort(key=lambda x: x.order)
        if not skip_path1:
            self.goal_ids.add(values[self.item1].id)
        self.goal_ids.add(values[self.item2].id)

        # self.blocks_to_move = []
        for goal in self.goals:
            self.generate_blocks(goal, self.blocks_to_move)

        # for node in self.need_to_visit_again:
        #     if node in path1:
        #         # MERGE = True
        #
        #         index_in_path = path1.index(node)
        #         node_in_path = path1[index_in_path]
        #
        #         if None in node_in_path.direction or None in node.direction:
        #             self.all_nodes[node.id] = (node, None)
        #             self.nodes_to_visit.put(node)
        #             continue
        #         node_in_path.direction = node_in_path.direction.union(node.direction)
        #         node_in_path.children = node_in_path.children.union(node.children)
        #         # for direction in node_in_path.num_blocks:
        #         #     node_in_path.num_blocks[direction] += node.num_blocks[direction]
        #         node_in_path.num_blocks += node.num_blocks
        #     else:
        #         self.all_nodes[node.id] = (node, None)
        #         self.nodes_to_visit.put(node)

        # time.sleep(4) #TODO: Remove Sleep time here
        for node in path1:
            self.all_nodes[node.id] = (node, None)
            self.nodes_to_visit.put(node)
        # print(self.nodes_to_visit)

        self.need_to_visit_again = []

    def notify_children(self, node):
        if node.children is None:
            print("No children to notify, confintuuing")
        else:
            for child in node.children:
                if child is None:
                    print("No children to notify, continuing")
                    continue
                else:
                    notify_node, robot = self.all_nodes[child]  #
                    # print(f"Notifiying child node with id: {child} {robot.id}")
                    self.currently_working.append(notify_node)

                    # try:
                    #     print(f"Removing blocks with node id: {node.id}")
                    #     print(blocks_to_move)
                    #     old_blocks = blocks_to_move
                    #
                    #     new_blocks = []
                    #     for block in old_blocks:
                    #         block.location = block.next_destination
                    #         block.next_destination = (notify_node.pos[0], notify_node.pos[1], 1)
                    #
                    #         new_block = Block(location=block.location,
                    #         next_destination=block.next_destination,
                    #                           final_destination=block.final_destination, id=block.id)
                    #         new_blocks.append(new_block)
                    #
                    #
                    #     next_id = currently_working[1].id #TODO: REMOVE THIS AND GET THE ID THE RIGHT WAY
                    #
                    #     blocks_to_move = new_blocks
                    #     # currently_claimed_set.remove
                    #
                    #     print(blocks_to_move)
                    #     # print(blocks_to_move[notify_node.id])
                    # except KeyError:
                    #     print("Key error, unable to remove blocks")
                    #     print(blocks_to_move)
                    #     print(node.id)
                    #     continue

    def update_blueprint_with_moved_blocks(self, moved_blocks):
        for block in moved_blocks:
            if block.placed_block:
                self.blueprint[block.location] = 1
            else:
                self.blueprint[block.location] = 0

    def wait_for_task_completion(self, robots, task_type="Ferry"):
        message = None
        finished = False
        finished_robots = set()
        num_robots_waiting = len(robots)
        while len(finished_robots) != num_robots_waiting:
            print("Waiting for robots to finish")
            messages = self.robot_communicator.get_communication()
            for update in messages:
                topic, received_message = update
                message = received_message.payload
                # print(type(message))
                # print(type(FerryBlocksStatusFinished))
                # print(f"Is instance: {isinstance(message, FerryBlocksStatusFinished)}")
                if task_type == "MOVE":
                    finished = isinstance(message, MovingFinished)
                else:
                    finished = isinstance(message, FerryBlocksStatusFinished)
                if finished:
                    finished_robots.add(topic)
                    if task_type != "MOVE":
                        moved_blocks = message.blocks_moved
                        self.update_blueprint_with_moved_blocks(moved_blocks)
                    # break

                if len(finished_robots) == num_robots_waiting:
                    break
                # print(f"[Structure] got message(s) from robot {topic}-> {message}")
                # print(received_message.robot_status)
                # print(f"DOING WORK on node: {node.id}")
            time.sleep(1)
        logger.debug(finished_robots)
        print("\n\n\n\nRobot has finished")
        time.sleep(3)

    def build_structure(self, level):
        self.divisions, self.wavefront_blueprint = self.buildingPlanner.create_divisions(
            division_size=self.division_size, level=level
        )
        self.item1 = -1
        self.item2 = 0
        logger.debug(f"Divisions: {self.divisions}")
        self.need_to_visit_again = []
        while self.item2 < len(self.divisions) - 1:

            self.merge_paths()
            # self.print_structure_status()
            # order

            robots = [
                Robot(id=b"ROBOT_1", pos=(0.5, 0.5), claimed_division=1),
                Robot(id=b"ROBOT_2", pos=(3.5, 0.5), claimed_division=2),
                Robot(id=b'ROBOT_3', pos=(4.5, 4.5), claimed_division=3),
                Robot(id=b'ROBOT_4', pos=(3.5, 2.5), claimed_division=4),
            ]

            # for i in need_to_visit_again:
            #     self.nodes_to_visit.put(i)
            #     need_to_visit_again.remove(i)

            self.currently_claimed_set.clear()  # TODO: May not want to clear list

            for bot in range(len(robots)):
                if self.nodes_to_visit.empty():
                    break
                else:
                    node = self.nodes_to_visit.get()
                    already_in = True
                    while already_in:
                        # print("Getting new")
                        # print(f"{self.currently_claimed_set}")
                        if len(self.currently_claimed_set) <= 0:
                            break
                        for i in self.currently_claimed_set:
                            if i.id == node.id:
                                # print("Node already inside claimed set, getting new one")
                                self.need_to_visit_again.append(node)
                                node = self.nodes_to_visit.get()
                            else:
                                # print("Found new node")
                                already_in = False
                                break
                        if not already_in:
                            break
                    self.currently_claimed_set.append(node)

            # print(currently_claimed_set)

            if len(self.currently_claimed_set) != len(robots):
                print("Number of points does not match number of robots")
                print(
                    f"Currently claimed set: {len(self.currently_claimed_set)}")
                print(f"Number of robots: {len(robots)}")
                print(robots)
                print(self.currently_claimed_set)

                currently_claimed_ids = [
                    division.id for division in self.currently_claimed_set]
                possible_divisions = list(self.divisions.values())
                possible_divisions.sort(key=lambda x: x.order)
                while len(self.currently_claimed_set) < len(robots):
                    try:
                        selected_division = possible_divisions.pop(0)
                    except IndexError:
                        raise Exception(
                            "Unable to assign divisions to all robots. Something went wrong")
                    if selected_division.id not in currently_claimed_ids:
                        self.currently_claimed_set.append(selected_division)

            assign_robots_closest_point(
                robots,
                self.currently_claimed_set,
                self.robot_communicator,
                blueprint=self.template,
            )
            #
            # assign robots
            self.wait_for_task_completion(robots=robots, task_type="MOVE")
            logger.error("Robots have reached destinations")
            # while True:
            #     pass
            # assign_robots_closest_point(robots, self.currently_claimed_set, None)
            for bot in robots:  # update dictionary to include robot with claimed division
                node, _ = self.all_nodes[bot.target.id]
                self.all_nodes[bot.target.id] = (node, bot)

            node1 = self.currently_claimed_set[0]
            self.currently_working.append(node1)

            ferry_blocks = []
            logger.debug(len(self.currently_working))
            while len(self.currently_working) > 0:
                # print(repr(node), end="\n\n")
                # TODO: TELL FIRST ROBOT TO START DOING WORK

                num_robots_assigned = 0
                for node in self.currently_working:
                    if num_robots_assigned > len(robots):
                        continue

                    # time.sleep(3)
                    else:
                        num_robots_assigned += 1
                        node, robot = self.all_nodes[node.id]

                        if robot is None:
                            raise Exception("Robot is none, line 288")
                        # if len(node.children) > 1:
                        #     blocks_to_move = get_blocks_to_move()

                        # flattened = [val for sublist in list(blocks_to_move.values()) for val in sublist]
                        #
                        # ferry_blocks_id = list(filter(lambda x: x.assigned_node == node.id, flattened))
                        # ferry_blocks_id.reverse()
                        #
                        # if len(node.children) > 1:
                        #     ferry_blocks = ferry_blocks_id[0:19]
                        # else:
                        #     ferry_blocks = ferry_blocks_id[0:9]
                        # ferry_blocks.reverse()

                        logger.info(f"Setting up blocks for node: {node}")
                        logger.info(f"Goal nodes: {self.goals}")
                        if node.id in self.goal_ids:
                            filtered_blocks = list(
                                filter(
                                    lambda x: x.final_destination == node.id,
                                    self.blocks_to_move,
                                )
                            )
                            ferry_blocks = self.get_new_block_location(
                                node, filtered_blocks, type="BUILD"
                            )

                            for block in ferry_blocks:
                                if block.location[0] == 0.5 and block.location[1] == 0.5:
                                    self.simulator_communicator.send_communication(
                                        topic=block.id,
                                        message=BlockLocationMessage(
                                            block_id=block.id,
                                            location=(
                                                block.location[0],
                                                block.location[1] + 0.5,
                                                block.location[2] + 0.5,
                                            ),
                                        ),
                                    )

                            # ferry_blocks = self.blocks_to_move
                            # ferry_blocks.reverse()
                            self.robot_communicator.send_communication(
                                topic=robot.id,
                                message=BuildMessage(
                                    blocks_to_move=ferry_blocks, blueprint=self.blueprint
                                ),
                            )

                            # self.blocks_to_move.
                            for i in ferry_blocks:
                                for j in self.blocks_to_move:
                                    if i.id == j.id:
                                        self.blocks_to_move.remove(j)
                                # self.blocks_to_move.remove(i)
                        else:
                            self.blocks_to_move = self.get_new_block_location(
                                node, self.blocks_to_move, type="FERRY"
                            )
                            ferry_blocks = self.blocks_to_move

                            for block in ferry_blocks:
                                if block.location[0] == 0.5 and block.location[1] == 0.5:
                                    self.simulator_communicator.send_communication(
                                        topic=block.id,
                                        message=BlockLocationMessage(
                                            block_id=block.id,
                                            location=(
                                                block.location[0],
                                                block.location[1] + 0.5,
                                                block.location[2] + 0.5,
                                            ),
                                        ),
                                    )

                            self.robot_communicator.send_communication(
                                topic=robot.id,
                                message=FerryBlocksMessage(
                                    blocks_to_move=ferry_blocks, blueprint=self.blueprint
                                ),
                            )

                        logger.info(f"Got new block location: {self.blocks_to_move}")

                        logger.debug(
                            f"Sending robot {robot.id} message to ferry blocks: {ferry_blocks}"
                        )

                        # for blocks in ferry_blocks:
                        #     blocks.assigned_node = node.id
                        """
                        if node.id in goal_ids:
                            structure.robot_communicator.send_communication(topic=robot.id, message=BuildMessage(
                                blocks_to_move=blocks_to_move[node.id]))
                        else:
                            flattened = [val for sublist in list(blocks_to_move.values()) for val in sublist]
                            ferry_blocks_id = list(filter(lambda x: x.assigned_node == node.id, flattened))
                            ferry_blocks_id.reverse()
                            ferry_blocks = ferry_blocks_id[0:2]
                            ferry_blocks.reverse()
                            structure.robot_communicator.send_communication(topic=robot.id, message=FerryBlocksMessage(
                            blocks_to_move=ferry_blocks))
                            print(f"Sending robot {robot.id} message to ferry blocks: {ferry_blocks}")
                        """
                        print(f"Node in currently_working: {node}")
                        print(f"\nDOING WORK on node: {node.id}")
                        print(f"\nDOING WORK on node: {node.id}")
                        print(f"\nDOING WORK on node: {node.id}")
                        print(f"\nDOING WORK on node: {node.id}")

                self.wait_for_task_completion(robots=robots, task_type="FERRY")

                # time.sleep(2)
                logger.info("Robot has finished working on node")
                # Robot has finished working
                try:
                    logger.debug("Getting new node")
                    new_node = self.nodes_to_visit.get(
                        timeout=1
                    )  # TODO: May need to check if id already in here
                    logger.debug(f"New node: {new_node}")
                    self.currently_claimed_set.append(new_node)
                    # currently_claimed_set.remove(node)
                    logger.debug("Done updating")
                except (Empty):
                    logger.debug("Queue is empty, continuing")
                    # continue

                logger.info("\nAssigning robots to points")

                # TODO: Change assign_robots to deal with if num of points less than number of robots
                if len(self.currently_claimed_set) > len(robots):
                    try:
                        self.currently_claimed_set.remove(node)
                    except ValueError:
                        print(
                            "Unable to remove node from currenly claimed set, continuing..."
                        )
                #
                # print(f"Currently Claimed Set: {len(currently_claimed_set)} Number of robots: {len(robots)}")
                # print(f"Current claimed set: {currently_claimed_set}")

                if len(self.currently_claimed_set) != len(robots):
                    print("Number of points does not match number of robots")
                    print(
                        f"Currently claimed set: {len(self.currently_claimed_set)}")
                    print(f"Number of robots: {len(robots)}")
                    raise Exception("Points not equal")

                assign_robots_closest_point(
                    robots,
                    self.currently_claimed_set,
                    self.robot_communicator,
                    blueprint=self.template,
                )
                # assign_robots_closest_point(robots, self.currently_claimed_set, None)
                time.sleep(2)
                logger.debug("Updating dictionary with new robot positions")
                for (
                    bot
                ) in robots:  # update dictionary to include robot with claimed division
                    update_node, _ = self.all_nodes[bot.target.id]
                    self.all_nodes[bot.target.id] = (update_node, bot)
                print("\n\n")

                # for node in currently_claimed_set:
                self.notify_children(node)

                # try:
                # self.currently_working.remove(node)
                for index, val in enumerate(self.currently_working):
                    if val.id == node.id:
                        self.currently_working.pop(index)
                        break

                # for index, val in enumerate(self.currently_claimed_set):
                #     if val.id == node.id:
                #         self.currently_claimed_set.pop(index)
                #         break

                for index, val in enumerate(self.goals):
                    if val.id == node.id:
                        self.goals.remove(val)
                        break

                for index, val in enumerate(self.goal_ids):
                    if val == node.id:
                        self.goal_ids.remove(val)
                        self.already_visited_ids.append(val)
                        break
                # except ValueError:
                #     print("Unable to remove node from currently working")
                #     print(f"Currently Working: {self.currently_working}")
                #     print(f"Current Node: {node}")
                #     print(f"Currently Claimed: {self.currently_claimed_set}")
                # try:
                # self.goals.remove(node)
                # self.goal_ids.remove(node.id)

                # except:
                #     print("")

    def generate_blocks(self, division, list):
        for blocks in list:
            if (
                blocks.final_destination == division.id
            ):  # TODO: Check to make sure this does not duplicate blocks
                return
        num_blocks_to_generate = division.num_blocks
        goal_id = division.id
        new_blocks = []
        for i in range(num_blocks_to_generate):
            new_blocks.append(Block(final_destination=goal_id))
        list.extend(new_blocks)

    def get_new_block_location(self, node, blocks_to_move, type="BUILD"):
        # node.direction = {'RIGHT'}

        logger.debug("In Get new block location")
        x_start, x_end = node.x_range
        y_start, y_end = node.y_range
        z_start, z_end = node.z_range

        level = self.template[x_start:x_end, y_start:y_end]
        m, n, _ = level.shape
        num_blocks = {"FRONT": 0, "RIGHT": 0, "BACK": 0, "LEFT": 0}

        num_directions = len(node.direction)
        for direction in node.direction:
            # if direction is None:
            #     direction = ('RIGHT')
            #     node.direction = direction
            num_blocks[direction] = int(node.num_blocks / num_directions)
            # num_blocks[direction] = node.num_blocks[direction]
            # TODO: Change so that this is not directly half, but the actual num of blocks for each division

        x_offset = node.x_range[0]
        y_offset = node.y_range[0]
        z_offset = node.z_range[0]

        if type == "BUILD" or None in node.direction:
            rows = len(level)
            columns = len(level[0])
            logger.debug(level)
            logger.debug(level.reshape((rows, columns)))
            logger.debug(self.template)
            layer, new_block_locations = spiral_sort_helper(
                rows,
                columns,
                level.reshape((rows, columns)),
                x_offset=x_offset,
                y_offset=y_offset,
                z_offset=z_offset,
            )
        else:
            logger.debug("Getting ferry block locations")
            logger.debug(f"Num blocks to ferry: {num_blocks}")
            logger.debug(f"Directions ferrying for: {node.direction}")

            _, _, new_block_locations = determine_ferry_regions(
                level,
                num_rows=m,
                num_cols=n,
                direction=node.direction,
                ferry_region_size=self.ferry_region_size,
                x_offset=x_offset,
                y_offset=y_offset,
                z_offset=z_offset + 1,  # TODO: Remove this extra +1
                num_blocks=num_blocks,
            )

        logger.debug("Got new block location in get new block")
        new_blocks = []
        # new_blocks = blocks_to_move
        blocks_to_move.reverse()
        for block, location in zip(blocks_to_move, new_block_locations):
            # index = blocks_to_move.index(block)
            # block = blocks_to_move[index]
            # try:
            #     location = new_block_locations[index]
            # except IndexError:
            #     print(f"\n\n\n\nGetting new block locations: Should show correct locations {new_blocks}")
            #     return new_blocks

            id = deepcopy(block.id)
            new_block = Block(final_destination=block.final_destination, id=id)
            new_block.location = block.location
            new_block.next_destination = block.next_destination

            new_block.set_next_location(location)
            # block.location = location
            logger.debug(new_block, location)
            # blocks_to_move[index] = block
            new_blocks.append(new_block)

        # print(new_blocks)
        # blocks_to_move = new_blocks
        logger.debug(
            f"\n\n\n\nGetting new block locations: Should show correct locations {new_blocks}"
        )

        # time.sleep(3)
        return new_blocks


if __name__ == "__main__":
    time.sleep(8)
    blueprint_status = BluePrintFactory().get_blueprint("Plane_10x10x2").data

    division_size = 5
    structure = StructureMain(
        template=blueprint_status[:, :, 0],
        division_size=division_size,
        blueprint=blueprint_status,
    )
    structure.initialize_communications(colors=None)

    x, y, num_of_levels = blueprint_status.shape
    for i in range(num_of_levels):
        logger.info(f"STARTING LEVEL: {i}")
        blueprint = blueprint_status[:, :, i]
        if len(blueprint.shape) < 3:
            x, y = blueprint.shape
            blueprint = blueprint.reshape((x, y, 1))
        structure.reset_building_planner(blueprint, division_size)
        structure.build_structure(level=i)
