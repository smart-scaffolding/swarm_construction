
from queue import PriorityQueue, Empty
from queue import Queue
from functools import total_ordering
# from priorityQueue import *
from structure.behaviors.building.assign_robots_min_distance import *
import time
from random import randint

@total_ordering
class Node:
    def __init__(self, wavefront_order, id, child, direction, pos, status="UNCLAIMED", num_blocks=9):
        self.order = wavefront_order
        self.id = id
        self.children = {child} #used for notifying other divisions when done
        self.direction = {direction} #expected to be only right and up
        self.status = status
        self.num_blocks = num_blocks #number of blocks to place in goal node
        self.pos = pos

    def __eq__(self, other):
        return self.order == other.order and self.id == other.id

    def __lt__(self, other):
        return self.order < other.order

    def __repr__(self):
        return f"\nID: {self.id}\nOrder: {self.order}\n\tChildren: {self.children}\n\tDirection: {self.direction}\n\t"\
            f"Num Blocks: {self.num_blocks}\n\tPosition: {self.pos}"

    def update_status(self, status):
        self.status = status

# path1 = [(2, 1), (3,2), (4,5), (5,8)]
# path2 = [(2, 1), (3,2), (4, 3), (5,6)]
# path1 = [(2, 1), (3,2), (4,5), (5,8)]
# path2 = [(2, 1), (3,2), (4,5), (5,6)]

path1 = [Node(wavefront_order=2, id=1, child=2, direction="RIGHT", pos=(0, 0)),
         Node(3, 2, 5, "UP", pos=(0, 1)),
         Node(4, 5, 8, "UP", pos=(1, 1)),
         Node(5, 8, None, None, pos=(2, 1))]
path2 = [Node(2, 1, 2, "RIGHT", pos=(0, 0)), Node(3, 2, 5, "UP", pos=(0, 1)), Node(4, 5, 6, "RIGHT", pos=(1, 1)),
         Node(5, 6,None,None, pos=(1, 2))]


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

goals = [Node(5, 8, None, None, pos=(2, 1)), Node(5, 6, None, None, pos=(1, 2))]
goals.sort(key=lambda x: x.order)
# print(goals)

for node in path1:
    all_nodes[node.id] = (node, None)
    q.put(node)

robots = [
          Robot(id=1, pos=(0, 0), claimed_division=1),
          Robot(id=2, pos=(0, 1), claimed_division=2),
          Robot(id=4, pos=(0, 2), claimed_division=3),
          Robot(id=3, pos=(1, 1), claimed_division=5),
          Robot(id=5, pos=(2, 2), claimed_division=9),
          ]

# print(f"All nodes values: {list(all_nodes.values())}")
# assign_robots_closest_point(robots, all_nodes.values()[0])

currently_claimed_set = [] #TODO: Ensure this data structure cannot be modified, important to preserve order

for bot in range(len(robots)):
    if q.empty():
        break
    else:
        node = q.get()
        currently_claimed_set.append(node)

# print(currently_claimed_set)




assign_robots_closest_point(robots, currently_claimed_set)  # assign robots
for bot in robots:  # update dictionary to include robot with claimed division
    node, _ = all_nodes[bot.target.id]
    all_nodes[bot.target.id] = (node, bot)

currently_working = []
node1 = currently_claimed_set[0]
currently_working.append(node1)

while len(currently_working) > 0:
    # print(repr(node), end="\n\n")
    #TODO: TELL FIRST ROBOT TO START DOING WORK

    for node in currently_working:
        # time.sleep(randint(1, 3))
        print(f"Node in currently_working: {node}")
        print(f"\nDOING WORK on node: {node.id}")
        print(f"\nDOING WORK on node: {node.id}")
        print(f"\nDOING WORK on node: {node.id}")
        print(f"\nDOING WORK on node: {node.id}")




        #Robot has finished working
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
        assign_robots_closest_point(robots, currently_claimed_set)
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
                notify_node, robot = all_nodes[child] #
                print(f"Notifiying child node with id: {child} {robot.id}")
                currently_working.append(notify_node)
                # currently_claimed_set.remove
        currently_working.remove(node)



