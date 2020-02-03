from components.structure import BuildingStates
from queue import PriorityQueue
from functools import total_ordering

@total_ordering
class Division:
    def __init__(self, status, position, id):
        if not isinstance(status, BuildingStates):
            raise Exception("Status must be set as a Building State status")
        self.status = status
        self.position = position
        self.id = id

    def _is_valid_operand(self, other):
        return hasattr(other, "status")

    def __eq__(self, other):
        if not self._is_valid_operand(other):
            return NotImplemented
        return self.status == other.status

    def __lt__(self, other):
        if not self._is_valid_operand(other):
            return NotImplemented
        return self.status < other.status

# level = [[Division(BuildingStates.EMPTY, (2, 0), 7), Division(BuildingStates.EMPTY, (2, 1), 8),
#           Division(BuildingStates.EMPTY, (2, 2), 9)],
#          [Division(BuildingStates.EMPTY, (1, 0), 4), Division(BuildingStates.EMPTY, (1, 1), 5),
#           Division(BuildingStates.EMPTY, (1, 2), 6)],
#          [Division(BuildingStates.DONE_ORIGIN, (0, 0), 1), Division(BuildingStates.EMPTY, (0, 1), 2),
#           Division(BuildingStates.EMPTY, (0, 2), 3)]
#          ]
#
# q = PriorityQueue()
# for row in level:
#     for division in row:
#         q.put(division)
#
# while not q.empty():
#     item = q.get()
#     print(f"ID: {item.id} STATUS: {item.status} POS: {item.position}")

@total_ordering
class Robot:
    def __init__(self, id, pos, claimed_division=None):
        self.id = id
        self.pos = pos
        self.closest_points = []
        self.target = None
        self.desired_target = None
        self.desired_target_distance = None
        self.claimed_division = claimed_division

    def find_new_target(self, points):
        # print(f"Unfiltered closest points: {self.closest_points}")
        self.closest_points.remove((self.desired_target_distance, self.desired_target))
        # print(f"Filtered closest points: {self.closest_points}")

        arg_min_score = np.argmin(np.array(self.closest_points)[:, 0])
        self.desired_target = self.closest_points[arg_min_score][1]
        self.desired_target_distance = self.closest_points[arg_min_score][0]
        # print(f"New target: {self.desired_target.pos}")

    def _is_valid_operand(self, other):
        return hasattr(other, "desired_target_distance")

    def __eq__(self, other):
        if not self._is_valid_operand(other):
            return NotImplemented
        return self.desired_target_distance == other.desired_target_distance

    def __lt__(self, other):
        if not self._is_valid_operand(other):
            return NotImplemented
        # if self.desired_target_distance == other.desired_target_distance:
        #     count = 1
        #     while self.closest_points[count][1].pos == other.closest_points[count][1].pos:
        #         count+=1
        #     return self.closest_points[count][1].pos < other.closest_points[count][1].pos

        return self.desired_target_distance < other.desired_target_distance

    def __repr__(self):
        return f"\n Robot ID: {self.id}\n\tPos: {self.pos}\n\tClaimed: {self.claimed_division}"

class Location:
    def __init__(self, id, pos, status="UNCLAIMED"):
        self.id = id
        self.pos = pos
        self.status = status

    def update_status(self, status):
        self.status = status


import math
import numpy as np
def distance(me, other):
    dist = math.sqrt((other[0] - me[0])**2 + (other[1] - me[1])**2)
    return dist

# robots = [Robot(1, (0, 0)), Robot(2, (0, 1)), Robot(3, (1, 0))]
# points = [Location(2, (0, 1)), Location(3, (0, 2)), Location(5, (1, 1))]


def robots_distances_to_locations(robots, points):

    for robot in robots:
        # print(f"Robot {robot.id} at: {robot.pos}")
        robot.closest_points = []
        for point in points:
            dist = distance(robot.pos, point.pos)
            robot.closest_points.append((dist, point))
            # print(f"\tLocation: {point.id} Point: {point.pos} Distance: {dist}")
        robot.closest_points = sorted(robot.closest_points, key=lambda point: point[1].pos,reverse=True)
        closest_points = np.array(robot.closest_points)[:, 0]
        arg_min_score = np.argmin(closest_points)
        robot.desired_target = robot.closest_points[arg_min_score][1]
        robot.desired_target_distance = robot.closest_points[arg_min_score][0]
    # print("\t" + 30*"-")
    # print(f"\tDesired target (id={robot.desired_target.id}): {robot.desired_target.pos}")
    # print(f"\t\tDistance: {robot.desired_target_distance}")
    # print("\n")

def assign_robots_closest_point(robots, points):
    robots_distances_to_locations(robots, points)
    q = PriorityQueue()
    for robot in robots:
        q.put((-robot.desired_target_distance, robot))

    # sorted_robots = sorted(robots, key=lambda bot: bot.desired_target_distance, reverse=True)
    print(30*"-")
    print("CLAIMING ROBOTS")
    claimed = []
    while not q.empty():
        dist, robot = q.get()
        index = points.index(robot.desired_target)
        if points[index] not in claimed:
            robot.target = robot.desired_target
            robot.claimed_division = robot.desired_target
            # points[index].status = "CLAIMED"
            claimed.append(points[index])
            print(f"Robot {robot.id} at: {robot.pos}")
            print("\t" + 30 * "-")
            print(f"\tClaimed target (id={robot.target.id}): {robot.target.pos}")
            print(f"\t\tDistance: {robot.desired_target_distance}")
            print("\n")

        else:
            robot.find_new_target(points)
            q.put((-robot.desired_target_distance, robot))

if __name__ == '__main__':
    robots = [Robot(1, (0, 0)), Robot(2, (0, 1)), Robot(4, (0, 2)), Robot(3, (1, 1))]
    points = [Location(1, (0, 0)), Location(2, (0, 1)), Location(5, (1, 1)), Location(6, (1, 2))]

    # robots_distances_to_locations(robots, points)
    assign_robots_closest_point(robots, points)