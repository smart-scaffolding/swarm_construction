import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import numpy as np
import heapq
import logging
from structure.pathplanning.path_planner import PathPlannerImp
import pandas as pd
from collections import OrderedDict

# coord_pairs = np.array([
#     [[1, 1, 1], [1, 0, 0], [1, 0, 0]],
#     [[1, 0, 0], [1, 0, 0], [1, 0, 0]],
#     [[1, 1, 0], [1, 1, 0], [0, 0, 0]],
#     [[1, 1, 1], [1, 0, 0], [0, 0, 0]],
#     [[1, 0, 0], [1, 0, 0], [0, 0, 0]],
#     [[1, 1, 0], [0, 0, 0], [0, 0, 0]]
# ])
#
# building_implemented = np.array([
#     [[1, 1, 1], [1, 0, 0], [0, 0, 0]],
#     [[1, 0, 0], [1, 0, 0], [0, 0, 0]],
#     [[1, 1, 0], [0, 0, 0], [0, 0, 0]],
#     [[1, 1, 1], [1, 0, 0], [0, 0, 0]],
#     [[1, 0, 0], [1, 0, 0], [0, 0, 0]],
#     [[1, 1, 0], [0, 0, 0], [0, 0, 0]]
# ])


class AStar(PathPlannerImp):
    def __init__(self, blueprint):
        self.blueprint = blueprint
        self.building_dimensions = self.blueprint.shape
        print("\nBuilding Dimensions: {}\n".format(self.building_dimensions))
        self.colors = np.array([[['#424ef5']*self.building_dimensions[2]] *
                   self.building_dimensions[1]]*self.building_dimensions[0])

        self.route = None

        self.logger = logging.getLogger('PathPlanning')

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


    def astar(self, array, start, goal):

        neighbors = [(0, 1, 0), (0, 1, -1), (0, 1, 1),
                     (0, -1, 0), (0, -1, -1), (0, -1, 1),
                     (1, 0, 0), (1, 0, -1), (1, 0, 1),
                     (-1, 0, 0), (-1, 0, -1), (-1, 0, 1),
                     (1, 1, 0), (1, 1, -1), (1, 1, 1),
                     (1, -1, 0), (1, -1, -1), (1, -1, 1),
                     (-1, 1, 0), (-1, 1, -1), (-1, 1, 1),
                     (-1, -1, 0), (-1, -1, -1), (-1, -1, 1)
                     ]

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data

            close_set.add(current)
            for i, j, k in neighbors:
                neighbor = current[0] + i, current[1] + j, current[2] + k
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:
                        if 0 <= neighbor[2] < array.shape[2]:
                            if array[neighbor[0]][neighbor[1]][neighbor[2]] == 0:
                                continue
                        else:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + \
                        self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))


    def get_path(self, start, goal):
        self.start = start
        self.goal = goal
        route = self.astar(self.blueprint, self.start, self.goal)
        if route is None:
            self.logger.error("Unable to find route between points {} and {}".format(self.start, self.goal))
            raise Exception("Path planning unable to find route")
        route = route + [self.start]
        route = route[::-1]
        print("Path to Traverse: {}\n".format(route))
        self.route = route
        return route

    def display_path(self):
        for i in self.route:
            self.colors[i] = '#ff0000ff'

        self.colors[self.route[-1]] = '#03fc62'
        return self.colors

