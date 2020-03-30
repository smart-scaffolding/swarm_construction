import time
from multiprocessing import Queue
from queue import PriorityQueue, Empty

import numpy as np

import components.structure.config as config
from components.simulator.model.graphics import vtk_named_colors
from components.structure.behaviors.building.assign_robots_min_distance import (
    assign_robots_closest_point,
)
from components.structure.behaviors.building.common_building import Block, Robot
from components.structure.behaviors.building.merge_paths import Node
from components.structure.pathplanning.searches.wavefront import Wavefront

configuration = None


class StructureMain:
    def __init__(self):
        self.configuration = config
        self.robot_queue = Queue()
        self.known_robots = {}


if __name__ == "__main__":

    structure = StructureMain()

    blueprint = np.array(
        [
            [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
            [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
            [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
            [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
            [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
            [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
        ]
    )

    bx, by, bz = blueprint.shape
    colors = [[[vtk_named_colors(["DarkGreen"])] * bz] * by] * bx

    blueprint = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]

    feeding_location = (0, 0)

    divisions = {
        (0, 0): Node(wavefront_order=1, id=1, pos=(1, 1)),
        (0, 1): Node(wavefront_order=2, id=2, pos=(4, 1)),
        (0, 2): Node(wavefront_order=3, id=3, pos=(7, 1)),
        (1, 0): Node(wavefront_order=2, id=4, pos=(1, 4)),
        (1, 1): Node(wavefront_order=3, id=5, pos=(4, 4)),
        (1, 2): Node(wavefront_order=4, id=6, pos=(7, 4)),
        (2, 0): Node(wavefront_order=3, id=7, pos=(1, 7)),
        (2, 1): Node(wavefront_order=4, id=8, pos=(4, 7)),
        (2, 2): Node(wavefront_order=5, id=9, pos=(7, 7)),
    }

    wf = Wavefront(
        blueprint=blueprint,
        feeding_location=feeding_location,
        furthest_division=(len(blueprint), len(blueprint[0])),
        print=False,
    )

    for division in divisions:

        node = divisions[division]
        path = wf.get_path(start=feeding_location, goal=division)
        path.append((division, None))
        # print(f"Got path to division {node.id}: {division} -> {path}")

        modified_path = []
        next_point = None
        for i in range(len(path)):
            pos, direction = path[i]

            next_point = None
            if i < len(path) - 1:
                next_point_location, _ = path[i + 1]
                next_point = divisions[next_point_location].id

            wavefront_order = divisions[pos].order
            node_id = divisions[pos].id

            actual_pos = divisions[pos].pos

            new_node = Node(
                wavefront_order=wavefront_order,
                id=node_id,
                pos=actual_pos,
                child=next_point,
                direction=direction,
            )

            previous_point = node_id

            modified_path.append(new_node)

        old_node = divisions[division]
        old_node.path_to_node = modified_path
        divisions[pos] = old_node
        # print(f"Modified path: {modified_path}")

    # print("\n\n\n")
    values = list(divisions.values())
    values.sort(key=lambda x: x.order)
    # print(f"Values: {values}")

    values.pop(0)  # TODO: Need to add back in first one (feeding location)

    item1 = 0
    item2 = 1
    while item2 < len(values) - 1:
        item1 = item1 + 1
        item2 = item2 + 1

        path1 = values[item1].path_to_node
        path2 = values[item2].path_to_node

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

        goals = [values[item1], values[item2]]

        goals.sort(key=lambda x: x.order)
        goal_ids = [values[item1].id, values[item2].id]

        blocks_to_move = [
            Block(
                location=(2, 0, 2),
                next_destination=(2, 0, 2),
                final_destination=(6, 0, 1),
            ),
            Block(
                location=(2, 1, 2),
                next_destination=(2, 1, 2),
                final_destination=(6, 1, 1),
            ),
            Block(
                location=(2, 2, 2),
                next_destination=(2, 2, 2),
                final_destination=(6, 2, 1),
            ),
            Block(
                location=(1, 0, 1),
                next_destination=(1, 0, 1),
                final_destination=(7, 0, 1),
            ),
            Block(
                location=(1, 1, 1),
                next_destination=(1, 1, 1),
                final_destination=(7, 1, 1),
            ),
            Block(
                location=(1, 2, 1),
                next_destination=(1, 2, 1),
                final_destination=(7, 2, 1),
            ),
            Block(
                location=(2, 0, 1),
                next_destination=(2, 0, 1),
                final_destination=(8, 0, 1),
            ),
            Block(
                location=(2, 1, 1),
                next_destination=(2, 1, 1),
                final_destination=(8, 1, 1),
            ),
            Block(
                location=(2, 2, 1),
                next_destination=(2, 2, 1),
                final_destination=(8, 2, 1),
            ),
            Block(
                location=(2, 0, 3),
                next_destination=(2, 0, 3),
                final_destination=(6, 0, 1),
            ),
            Block(
                location=(2, 1, 3),
                next_destination=(2, 1, 3),
                final_destination=(6, 1, 1),
            ),
            Block(
                location=(2, 2, 3),
                next_destination=(2, 2, 3),
                final_destination=(6, 2, 1),
            ),
            Block(
                location=(1, 0, 2),
                next_destination=(1, 0, 2),
                final_destination=(7, 0, 1),
            ),
            Block(
                location=(1, 1, 2),
                next_destination=(1, 1, 2),
                final_destination=(7, 1, 1),
            ),
            Block(
                location=(1, 2, 2),
                next_destination=(1, 2, 2),
                final_destination=(7, 2, 1),
            ),
            Block(
                location=(2, 0, 4),
                next_destination=(2, 0, 4),
                final_destination=(8, 0, 1),
            ),
            Block(
                location=(2, 1, 4),
                next_destination=(2, 1, 4),
                final_destination=(8, 1, 1),
            ),
            Block(
                location=(2, 2, 4),
                next_destination=(2, 2, 4),
                final_destination=(8, 2, 1),
            ),
        ]

        time.sleep(5)
        for node in path1:
            all_nodes[node.id] = (node, None)
            q.put(node)

        currently_claimed_set = (
            []
        )  # TODO: Ensure this data structure cannot be modified, important to preserve order

        robots = [
            Robot(id=b"ROBOT_1", pos=(1.5, 1.5), claimed_division=1),
            Robot(id=b"ROBOT_2", pos=(4.5, 1.5), claimed_division=2),
            # Robot(id=b'ROBOT_3', pos=(4.5, 4.5), claimed_division=3),
            # Robot(id=b'ROBOT_4', pos=(7.5, 1.5), claimed_division=4),
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

        assign_robots_closest_point(robots, currently_claimed_set, None)  # assign robots
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

                time.sleep(5)
                node, robot = all_nodes[node.id]

                ferry_blocks = blocks_to_move

                print(f"Sending robot {robot.id} message to ferry blocks: {ferry_blocks}")

                for blocks in ferry_blocks:
                    blocks.assigned_node = node.id

                print(f"Node in currently_working: {node}")
                print(f"\nDOING WORK on node: {node.id}")
                print(f"\nDOING WORK on node: {node.id}")
                print(f"\nDOING WORK on node: {node.id}")
                print(f"\nDOING WORK on node: {node.id}")

                message = None
                finished = False

                time.sleep(2)
                print("Robot has finished working on node")
                # Robot has finished working
                try:
                    print("Getting new node")
                    new_node = q.get(timeout=2)
                    print(f"New node: {new_node}")
                    currently_claimed_set.append(new_node)
                    # currently_claimed_set.remove(node)
                    print("Done updating")
                except (Empty):
                    print("Queue is empty, continuing")
                    # continue

                print("\nAssigning robots to points")

                # TODO: Change assign_robots to deal with if num of points less than number of robots
                if len(currently_claimed_set) > len(robots):
                    currently_claimed_set.remove(node)

                assign_robots_closest_point(robots, currently_claimed_set, None)
                time.sleep(5)
                print("Updating dictionary with new robot positions")
                for (
                    bot
                ) in robots:  # update dictionary to include robot with claimed division
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

                        try:
                            print(f"Removing blocks with node id: {node.id}")
                            print(blocks_to_move)
                            old_blocks = blocks_to_move

                            new_blocks = []
                            for block in old_blocks:
                                block.location = block.next_destination
                                block.next_destination = (
                                    notify_node.pos[0],
                                    notify_node.pos[1],
                                    1,
                                )

                                new_block = Block(
                                    location=block.location,
                                    next_destination=block.next_destination,
                                    final_destination=block.final_destination,
                                    id=block.id,
                                )
                                new_blocks.append(new_block)

                            next_id = currently_working[
                                1
                            ].id  # TODO: REMOVE THIS AND GET THE ID THE RIGHT WAY

                            blocks_to_move = new_blocks
                            # currently_claimed_set.remove

                            print(blocks_to_move)
                            # print(blocks_to_move[notify_node.id])
                        except KeyError:
                            print("Key error, unable to remove blocks")
                            print(blocks_to_move)
                            print(node.id)
                            continue
                currently_working.remove(node)


def get_blocks_to_move():
    pass
