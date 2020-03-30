from itertools import cycle
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np

from components.structure.behaviors.building.common_building import Division
from components.structure.behaviors.building.select_ferry_regions import (
    determine_ferry_regions,
)
from components.structure.pathplanning.searches.wavefront import Wavefront


class BuildingPlanner:
    def __init__(self, blueprint, feeding_location):
        self.blueprint = blueprint
        self.structure = None
        self.feeding_location = feeding_location

    def create_divisions(self, division_size=3, level=0):
        # TODO: Change so does not need to be square
        x, y, z = self.blueprint.shape
        colors = np.array([[["DarkGreen"] * z] * y] * x)
        if x != y:
            raise Exception("Structure must be square for the time being")

        vtk_colors = cycle(["DarkGreen", "Red", "Blue", "Orange", "Yellow"])
        increment = division_size
        p_zi = 0
        p_xi = 0
        p_yi = 0
        self.divisions = []

        if z == 1:
            for xi in range(increment, x + increment, increment):
                for yi in range(increment, y + increment, increment):
                    # create division
                    if yi + increment > y:
                        yi = y
                    if xi + increment > x:
                        xi = x

                    d = Division(
                        (p_xi, xi),
                        (p_yi, yi),
                        (level, level + 1),
                        num_blocks=np.sum(self.blueprint[p_xi:xi, p_yi:yi, 0:1]),
                    )
                    colors[p_xi:xi, p_yi:yi, 0:1] = next(vtk_colors)
                    self.divisions.append(d)
                    # print(yi)
                    p_yi = yi
                p_yi = 0
                p_xi = xi
            p_yi = 0
            p_xi = 0

        else:
            for zi in range(increment, z + increment, increment):
                for xi in range(increment, x + increment, increment):
                    for yi in range(increment, y + increment, increment):
                        # create division
                        if yi + increment > y:
                            yi = y
                        if xi + increment > x:
                            xi = x
                        if zi + increment > z:
                            zi = z
                        d = Division((p_xi, xi), (p_yi, yi), (p_zi, zi))
                        colors[p_xi:xi, p_yi:yi, p_zi:zi] = next(vtk_colors)
                        self.divisions.append(d)
                        # print(yi)
                        p_yi = yi
                    p_yi = 0
                    p_xi = xi
                p_yi = 0
                p_xi = 0
                p_zi = zi

        self._reshape_divisions_helper()
        self._assign_children_helper()
        return self.divisions, self.structure

    def _reshape_divisions_helper(self):
        reshape_value = int(sqrt(len(self.divisions)))
        reshaped = np.asarray(self.divisions).reshape((reshape_value, reshape_value))
        # print(reshaped.shape)

        x, y = reshaped.shape

        new_structure = {}
        self.structure = np.zeros((x, y))
        for x_val in range(x):
            for y_val in range(y):
                new_structure[(x_val, y_val)] = reshaped[x_val][y_val]
                self.structure[x_val][y_val] = 1
        # print(new_structure)
        print(self.structure)
        self.divisions = new_structure

    def _assign_children_helper(self):
        wf = Wavefront(
            blueprint=self.structure,
            feeding_location=(0, 0),
            furthest_division=(len(self.structure), len(self.structure[0])),
            divisions=self.divisions,
            print=False,
        )

        self.divisions[(0, 0)].order = 1  # TODO: May want to remove this
        print(self.divisions)
        for division in self.divisions:

            # node = self.divisions[division]
            path = wf.get_path(start=self.feeding_location, goal=division)
            path.append((division, None))
            # print(f"Got path to division {node.id}: {division} -> {path}")

            modified_path = []
            next_point = None
            for i in range(len(path)):
                pos, direction = path[i]

                # print(f"Pos: {pos}, Direction")
                next_point = None
                if i < len(path) - 1:
                    next_point_location, _ = path[i + 1]
                    next_point = self.divisions[next_point_location].id

                old_node = self.divisions[pos]
                # wavefront_order = old_node.order
                node_id = old_node.id

                x_range = old_node.x_range
                y_range = old_node.y_range
                z_range = old_node.z_range
                # actual_pos = old_node.pos
                # centroid = old_node.centroid
                num_blocks = old_node.num_blocks
                order = old_node.order

                # new_node = Node(wavefront_order=wavefront_order, id=node_id, pos=actual_pos, child=next_point,
                #                 direction=direction)

                new_node = Division(
                    x_range=x_range, y_range=y_range, z_range=z_range, id=node_id
                )

                # new_node = self.divisions[pos]
                new_node.num_blocks = num_blocks
                new_node.children.add(next_point)
                try:
                    if len(new_node.children) > 1 and None in new_node.children:
                        new_node.children.remove(None)
                except KeyError:
                    continue

                new_node.order = order
                new_node.direction.add(direction)

                try:
                    if len(new_node.direction) > 1 and None in new_node.direction:
                        new_node.direction.remove(None)
                except KeyError:
                    continue
                modified_path.append(new_node)

            original_node = self.divisions[division]
            original_node.path_to_node = modified_path
            self.divisions[pos] = original_node

        # print(self.divisions)

    def get_cmap(self, n, name="hsv"):
        """Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
        RGB color; the keyword argument name must be a standard mpl colormap name."""
        return plt.cm.get_cmap(name, n)

    def display(self, z_height=None, return_plot=False):
        x, y, z = self.blueprint.shape
        self.colors = np.array([[["w"] * z] * y] * x)
        # count = 0
        cycol = cycle("bgrcmyk")
        for division in self.divisions:
            # if count == 0:
            #     pass
            # count +=1
            x_start, x_end = division.x_range
            y_start, y_end = division.y_range
            z_start, z_end = division.z_range
            print(x_start, x_end, y_start, y_end, z_start, z_end)
            self.colors[x_start:x_end, y_start:y_end, z_start:z_end] = next(cycol)

        fig = plt.figure()
        ax = fig.gca(projection="3d")
        if z_height is None:
            display = self.blueprint
            disp_colors = self.colors
        else:
            display = self.blueprint[:, :, :z_height]
            disp_colors = self.colors[:, :, :z_height]
        ax.voxels(display, facecolors=disp_colors, edgecolors="k")  # brighter

        ax.set(xlabel="X", ylabel="Y", zlabel="Z")
        xlim, ylim, zlim = self.blueprint.shape
        ax.set_xlim(0, xlim)
        ax.set_ylim(0, ylim)
        ax.set_zlim(0, zlim)
        if return_plot:
            return plt, ax
        plt.show()


if __name__ == "__main__":
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
    blueprint = np.array([[[1] * 1] * 15] * 15)

    buildingPlanner = BuildingPlanner(blueprint)

    divisions, _ = buildingPlanner.create_divisions(division_size=5)
    # print(divisions)
    # x, y, z = blueprint.shape
    a, b = int(len(divisions) / 2), len(divisions) - int(len(divisions) / 2)
    reshape_value = int(sqrt(len(divisions)))
    reshaped = np.asarray(divisions).reshape((reshape_value, reshape_value))
    # print(reshaped.shape)

    x, y = reshaped.shape

    new_structure = {}
    wavefront_blueprint = np.zeros((x, y))
    for x_val in range(x):
        for y_val in range(y):
            new_structure[(x_val, y_val)] = reshaped[x_val][y_val]
            wavefront_blueprint[x_val][y_val] = 1
    # print(new_structure)
    print(wavefront_blueprint)

    wf = Wavefront(
        blueprint=wavefront_blueprint,
        feeding_location=(0, 0),
        furthest_division=(len(wavefront_blueprint), len(wavefront_blueprint[0])),
        divisions=new_structure,
        print=False,
    )

    goal = new_structure[(2, 1)].id

    path = wf.get_path(start=(0, 0), goal=(2, 1))
    path.append(((2, 1), None))
    print(path)

    for item in new_structure:
        print(new_structure[item])
        node = new_structure[item]
        node.direction = {"RIGHT"}

        x_start, x_end = node.x_range
        y_start, y_end = node.y_range
        z_start, z_end = node.z_range

        level = blueprint[x_start:x_end, y_start:y_end, z_start:z_end]
        m, n, _ = level.shape
        num_blocks = node.num_blocks
        x_offset, y_offset, z_offset = node.centroid

        _, new_block_locations = determine_ferry_regions(
            level,
            num_rows=m,
            num_cols=n,
            direction=node.direction,
            ferry_region_size=2,
            x_offset=round(x_offset) - 2,
            y_offset=round(y_offset) - 2,
            z_offset=round(z_offset),
            num_blocks={"FRONT": 0, "RIGHT": node.num_blocks, "BACK": 0, "LEFT": 0},
        )

        print(new_block_locations)
        # while True:
        #     pass

    # blueprint = np.array([[1, 2, 3, 4, 5], [6, 7, 8, 9, 10], [11, 12, 13, 14, 15], [16, 17, 18, 19, 20], [21, 22, 23,
    #                                                                                                       24, 25]])
    #
    # rows = len(blueprint)
    # columns = len(blueprint[0])
    # layer, new_pos = spiral_sort_helper(rows, columns, blueprint[:, :])
    # print(layer)
    # print(new_pos)

    # buildingPlanner.display()
