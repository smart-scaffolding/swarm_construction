from structure.pathplanning.path_planner import PathPlannerImp


class Wavefront(PathPlannerImp):
    def __init__(self, blueprint, furthest_division, feeding_location, print=False):
        self.blueprint = blueprint
        self.layer = self.convert_blueprint_to_layer(blueprint)
        self.feeding_location = feeding_location
        self.furthest_division = furthest_division
        self.initialize_wavefront(goal=self.feeding_location, start=self.furthest_division)
        self.print = print

        if print:
            self.layer.print_grid(grid=convert_layer_to_grid(self.blueprint, self.layer, self.feeding_location,
                                                             self.furthest_division))

    def convert_blueprint_to_layer(self, blueprint):
        positions = {}
        xdim = len(blueprint)
        ydim = len(blueprint[1])

        for y in range(ydim):
            for x in range(xdim):
                positions[(x, y)] = blueprint[x][y]

        return Layer(xdim, ydim, positions, blueprint)

    def initialize_wavefront(self, goal, start):
        heap = []
        new_heap = []
        x, y = goal
        last_wave = 3

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
            print("Goal is unreachable")
            return

    def get_path(self, start, goal):
        return self.layer.navigate_to_goal(start, goal, display=self.print)


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

        while not finished:
            x, y = self.pos
            self.positions[self.pos] = 'R'
            moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
            move_directions = ["Front", "Back", "Right", "Left"]

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
            print(f"Moved {move_directions[least_index]} {moves[least_index]}")

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


if __name__ == '__main__':
    blueprint = [
        [1, 0, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1],
    ]

    wf = Wavefront(blueprint=blueprint, feeding_location=(0, 0), furthest_division=(7, 7), print=True)

    path = wf.get_path(start=(0, 0), goal=(7, 7))

    print(f"\nPath: {path}")
