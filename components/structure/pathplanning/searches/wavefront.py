from components.structure.pathplanning.path_planner import PathPlannerImp
# from components.structure.behaviors.building.merge_paths import Node
# from components.structure.behaviors.divide_structure import Division
from queue import PriorityQueue, Empty
import time


class Wavefront(PathPlannerImp):
    def __init__(self, blueprint, furthest_division, feeding_location, divisions=None, print=False):
        self.blueprint = blueprint
        self.layer = self.convert_blueprint_to_layer(blueprint)
        self.feeding_location = feeding_location
        self.furthest_division = furthest_division
        self.initialize_wavefront(goal=self.feeding_location, start=self.furthest_division)
        self.print = print
        if divisions:
            self.map_wavefront_values_to_divisions(divisions)
        if print:
            self.layer.print_grid(grid=convert_layer_to_grid(self.blueprint, self.layer, self.feeding_location,
                                                             self.furthest_division))

    def map_wavefront_values_to_divisions(self, divisions):
        for position in self.layer.positions:
            try:
                divisions[position].order = self.layer.positions[position]
                # print(position)
                # print(self.layer.positions[position])
                # print(divisions[position].order)

            except KeyError:
                print(f"Unable to find key: {position} in divisions")
                continue
        # print(divisions)
        return divisions

    def convert_blueprint_to_layer(self, blueprint):
        positions = {}
        xdim = len(blueprint)
        ydim = len(blueprint[1])

        for y in range(ydim-1, -1, -1):
            for x in range(xdim-1, -1, -1):
                positions[(x, y)] = blueprint[x][y]

        # print(positions)
        return Layer(xdim, ydim, positions, blueprint)

    def initialize_wavefront(self, goal, start):
        heap = []
        new_heap = []
        x, y = goal
        last_wave = 1

        moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]

        for move in moves:
            x, y = move
            if x < 0 or y < 0 or x >= self.layer.xdim or y >= self.layer.ydim:
                continue

            if self.layer.positions[move] == 1:
                self.layer.positions[move] = 2
                heap.append(move)

        reached_start = False
        goal_layer = None
        goal_last_wave = None

        for current_wave in range(3, self.layer.xdim * self.layer.ydim):
            last_wave = last_wave + 1

            while heap != []:
                position = heap.pop()
                (x, y) = position
                moves = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

                for move in moves:
                    x, y = move

                    if x < 0 or y < 0 or x >= self.layer.xdim or y >= self.layer.ydim:
                        continue

                    if self.layer.positions[move] != 0:
                        if self.layer.positions[move] == 1 and self.layer.positions[position] == current_wave - 1:
                            self.layer.positions[move] = current_wave
                            new_heap.append(move)

                        if move == start:
                            reached_start = True
                            goal_layer = self.layer
                            goal_last_wave = last_wave

            heap = new_heap
            new_heap = []

        if reached_start:
            return goal_layer, goal_last_wave
        else:
            # print("Goal is unreachable")
            return

    def get_path(self, start, goal):
        x_start, y_start = start
        x_goal, y_goal = goal

        x_start = x_start % self.layer.xdim
        y_start = y_start % self.layer.ydim
        x_goal = x_goal % self.layer.xdim
        y_goal = y_goal % self.layer.ydim

        return self.layer.navigate_to_goal((x_start, y_start), (x_goal, y_goal), display=self.print)


class Layer(object):

    def __init__(self, xdim, ydim, positions, blueprint):
        self.xdim = xdim
        self.ydim = ydim
        self.positions = positions
        self.blueprint = blueprint

    def columnize(self, word, width, align='Left'):

        nSpaces = width - len(word)
        if len(word) == 1:
            word = " "+word
        if nSpaces < 0:
            nSpaces = 0
        if align == 'Left':
            return word + (" " * nSpaces)
        if align == 'Right':
            return (" " * nSpaces) + word
        return (" " * int(nSpaces / 2)) + word + (" " * int(nSpaces - nSpaces / 2))

    def print_grid(self, grid, column=10):
        print()
        print('%s%s' % (self.columnize('Table |', column * 2, 'Right'), \
                        '|'.join([self.columnize('Col %d ' % i, column, 'Center') \
                                  for i in range(0, len(grid[0]))])))
        spaces = sum([column for i in range(len(grid[0]))]) + column * 2
        print('=' * spaces)
        for i, item in enumerate(grid):
            print('%s%s' % (self.columnize('Row %d |' % (i), column * 2, 'Right'), \
                            '|'.join([self.columnize(str(num), column, 'Center') \
                                      for num in item])))

    def navigate_to_goal(self, goal, start, display=False):
        """
        :param start:
        :param goal:
        :param display:
        :return:
        """

        path = []
        self.pos = start
        finished = False

        current = self.positions[start]

        if goal == start:
            return []
        while not finished:
            x, y = self.pos

            # self.positions[self.pos] = 'R'
            moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
            move_directions = ["BACK", "FRONT", "RIGHT", "LEFT"]

            least_index = 1
            found_next_node = False
            for w in range(len(moves)):
                move = moves[w]
                x, y = move
                if x < 0 or y < 0 or x >= self.xdim or y >= self.ydim:
                    continue

                # try:
                if move == goal:
                    finished = True
                    least_index = w
                    found_next_node = True
                    break

                if self.positions[move] == current - 1:
                    self.least = self.positions[move]
                    least_index = w
                    found_next_node = True
                    break

            if not found_next_node:
                print("Unable to find path to goal")
                return
                # except:
                #     pass

            current = current - 1
            if display:
                self.positions[self.pos] = 'X' # TODO: Comment this out, as it is for visualization only

            path.append((moves[least_index], move_directions[least_index]))
            # print(f"Moved {move_directions[least_index]} {moves[least_index]}")

            self.pos = moves[least_index]  # This will be converted to "move robot in x direction"

        if display:
        # TODO: Comment this out, as it is for visualization only
            self.print_grid(convert_layer_to_grid(grid=self.blueprint, layer=self, goal=goal, start=start))

        path.reverse()
        return path

def convert_layer_to_grid(grid, layer, start, goal):
    for key, value in layer.positions.items():
        x, y = key

        if (x, y) == start:
            value = "G"

        if (x, y) == goal:
            value = "S"

        try:
            grid[x][y] = value
        except:
            pass
    return grid


# class Division:
#     def __init__(self, division_id, position):
#         self.division_id = division_id
#         self.path = None
#         self.position = position


if __name__ == '__main__':
    # blueprint = [
    #     [1, 0, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    # ]

    blueprint = [
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]
    ]

    feeding_location = (0, 0)

    # divisions = {
    #     (0, 0): Division(division_id=1, position=(1.5, 1.5)),
    #     (0, 1): Node(wavefront_order=2, id=2, pos=(4.5, 1.5)),
    #     (0, 2): Node(wavefront_order=3, id=3, pos=(7.5, 1.5)),
    #     (1, 0): Node(wavefront_order=2, id=4, pos=(1.5, 4.5)),
    #     (1, 1): Node(wavefront_order=3, id=5, pos=(4.5, 4.5)),
    #     (1, 2): Node(wavefront_order=4, id=6, pos=(7.5, 4.5)),
    #     (2, 0): Node(wavefront_order=3, id=7, pos=(1.5, 7.5)),
    #     (2, 1): Node(wavefront_order=4, id=8, pos=(4.5, 7.5)),
    #     (2, 2): Node(wavefront_order=5, id=9, pos=(7.5, 7.5)),
    # }
    #
    # wf = Wavefront(blueprint=blueprint, feeding_location=feeding_location, furthest_division=(len(blueprint),
    #                                                                                           len(blueprint[0])),
    #                print=False)
    #
    # for division in divisions:
    #
    #     node = divisions[division]
    #     path = wf.get_path(start=feeding_location, goal=division)
    #     path.append((division, None))
    #     print(f"Got path to division {node.id}: {division} -> {path}")
    #
    #
    #
    #
    #     modified_path = []
    #     next_point = None
    #     for i in range(len(path)):
    #         pos, direction = path[i]
    #
    #         next_point = None
    #         if i < len(path)-1:
    #             next_point_location, _ = path[i+1]
    #             next_point = divisions[next_point_location].id
    #
    #
    #         wavefront_order = divisions[pos].order
    #         node_id = divisions[pos].id
    #
    #         actual_pos = divisions[pos].pos
    #
    #         new_node = Node(wavefront_order=wavefront_order, id=node_id, pos=actual_pos, child=next_point, direction=direction)
    #
    #         previous_point = node_id
    #
    #         modified_path.append(new_node)
    #
    #     old_node = divisions[division]
    #     old_node.path_to_node = modified_path
    #     divisions[pos] = old_node
    #     print(f"Modified path: {modified_path}")
    #
    # print("\n\n\n")
    # values = list(divisions.values())
    # values.sort(key=lambda x: x.order)
    # print(f"Values: {values}")
    #
    # values.pop(0) # TODO: Need to add back in first one (feeding location)
    #
    #
    # item1 = 0
    # item2 = 1
    #
    # path1 = values[item1].path_to_node
    # path2 = values[item2].path_to_node
    #
    # all_nodes = {}
    #
    # # for point in path
    # merged_path = path1
    # q = PriorityQueue()
    #
    # # MERGE = False
    # for node in path2:
    #     if node in path1:
    #         # MERGE = True
    #
    #         index_in_path = path1.index(node)
    #         node_in_path = path1[index_in_path]
    #
    #         node_in_path.direction = node_in_path.direction.union(node.direction)
    #         node_in_path.children = node_in_path.children.union(node.children)
    #         node_in_path.num_blocks += node.num_blocks
    #
    #     else:
    #         all_nodes[node.id] = (node, None)
    #         q.put(node)
    #
    # goals = [values[item1], values[item2]]
    #
    # goals.sort(key=lambda x: x.order)
    # goal_ids = [values[item1].id, values[item2].id]
    #
    # for node in path1:
    #     all_nodes[node.id] = (node, None)
    #     q.put(node)
    #
    # currently_claimed_set = []  # TODO: Ensure this data structure cannot be modified, important to preserve order
    # # print(all_nodes)




