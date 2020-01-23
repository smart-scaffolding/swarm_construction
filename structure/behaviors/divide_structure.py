import numpy as np
# from .graphics import VtkPipeline
# from .graphics import axesCubeFloor
# from .graphics import MakeAxesActor
# from .graphics import setup_structure_display
from robopy.base.graphics import *
from robopy.base.states import DivisionStates
import time
import vtk
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from itertools import cycle

class BuildingPlanner:
    def __init__(self, blueprint, time_steps=3000, gif=None, num_robots=10, threshold=2):
        self.pipeline = VtkPipeline(total_time_steps=time_steps, gif_file=gif)
        self.param = {
            "cube_axes_x_bounds": np.matrix([[0, len(blueprint)]]),
            "cube_axes_y_bounds": np.matrix([[0, len(blueprint[0])]]),
            "cube_axes_z_bounds": np.matrix([[0, len(blueprint[0][0])]]),
            "floor_position": np.matrix([[0, 0, 0]])
        }
        self.blueprint = blueprint
        self.structure = None
        cube_axes = axesCubeFloor(self.pipeline.ren,
                                  self.param.get("cube_axes_x_bounds"),
                                  self.param.get("cube_axes_y_bounds"),
                                  self.param.get("cube_axes_z_bounds"),
                                  self.param.get("floor_position"))

        self.pipeline.add_actor(cube_axes)
        self.structure_actors = None
        self.NUM_ROBOTS = num_robots
        self.THRESHOLD = threshold

    def execute(self, obj, event):

        self.pipeline.timer_tick()
        timer = self.pipeline.timer_count
        print(timer)
        if len(self.structure_actors) > 0 and timer % 10 == 0:
            for robot in range(self.NUM_ROBOTS):
                block = self.structure_actors.pop()
                self.pipeline.add_actor(block)
            self.pipeline.animate()

        self.pipeline.iren = obj
        self.pipeline.iren.GetRenderWindow().Render()

    def create_building_plan(self):
        blueprint_sorted = self.structure.sort(key=lambda block: [block.position[0], block.position[1],
                                                                  block.position[2]])
        return blueprint_sorted

    def show_structure(self):

        self.structure = intialize_structure(self.blueprint)

        divided_structure = self.divide_structure()
        building_plan = []
        for division in divided_structure:
            building_plan.append(np.array(spiral_sort(division)).flatten())
        self.structure_actors = setup_structure_display(np.array(building_plan), self.NUM_ROBOTS)
        self.pipeline.iren.AddObserver('TimerEvent', self.execute)


        # if display_path:
        #     self.pipeline.add_actor(self._display_path())

        xyzLabels = ['X', 'Y', 'Z']
        scale = [1.0, 1.0, 1.0]
        axes = MakeAxesActor(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)
        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(self.pipeline.iren)
        om2.EnabledOn()
        om2.InteractiveOn()

        self.pipeline.animate()

    def create_divisions(self):
        #TODO: Change so does not need to be square
        x, y, z = self.blueprint.shape
        colors = np.array([[["DarkGreen"] * z] * y] * x)
        if x != y or y != z:
            raise Exception("Structure must be square for the time being")

        vtk_colors = cycle(["DarkGreen", "Red", "Blue", "Orange", "Yellow"])
        increment = 4
        p_zi = 0
        p_xi = 0
        p_yi = 0
        self.divisions = []

        for zi in range(increment, z+increment, increment):
            for xi in range(increment, x+increment, increment):
                for yi in range(increment, y+increment, increment):
                    #create division
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


        return self.divisions, colors

    def get_cmap(self, n, name='hsv'):
        '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
        RGB color; the keyword argument name must be a standard mpl colormap name.'''
        return plt.cm.get_cmap(name, n)

    def display(self, z_height=None, return_plot=False):
        x, y, z = self.blueprint.shape
        self.colors = np.array([[['w']*z]*y]*x)
        # count = 0
        cycol = cycle('bgrcmyk')
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
        ax = fig.gca(projection='3d')
        if z_height is None:
            display = self.blueprint
            disp_colors = self.colors
        else:
            display = self.blueprint[:, :, :z_height]
            disp_colors = self.colors[:, :, :z_height]
        ax.voxels(display,
                  facecolors=disp_colors,
                  edgecolors='k'  # brighter
                  )

        ax.set(xlabel='X', ylabel='Y', zlabel='Z')
        xlim, ylim, zlim = self.blueprint.shape
        ax.set_xlim(0, xlim)
        ax.set_ylim(0, ylim)
        ax.set_zlim(0, zlim)
        if return_plot:
            return plt, ax
        plt.show()

    def divide_structure(self):


        x, y, z = self.structure.shape

        select_colors = ['r', 'g', 'k', 'b']

        if z > y and z > x:
            maxDirection = "Z"
        elif y > x and y > z:
            maxDirection = "Y"
        else:
            maxDirection = "X"

        increment = int(max(x, y, z) / self.NUM_ROBOTS)
        print(f"Increment: {increment}")
        if increment <= self.THRESHOLD:
            raise Exception("Too many robots, please try lowering the number of robots")

        divided_structure = []
        start = 0
        end = increment
        scale = 1
        for robot in range(self.NUM_ROBOTS):
            if robot == self.NUM_ROBOTS - 1:
                end = None
            print(start, end, scale)
            if maxDirection == "X":
                divided_structure.append(self.structure[start:end, :, :])
            if maxDirection == "Y":
                divided_structure.append(self.structure[:, start:end, :])
            if maxDirection == "Z":
                divided_structure.append(self.structure[:, :, start:end])

            start = end
            scale += 1
            end = increment * scale

        return np.array(divided_structure)


def intialize_structure(blueprint):
    print(f"Blueprint size: {len(blueprint)} -- {len(blueprint[0])} -- {len(blueprint[0][0])}")

    structure = np.array([[[Block((0, 0, 0), False)]*len(blueprint[0][0])]*len(blueprint[0])]*len(blueprint))
    print(f"\n\nRANGE: {range(len(blueprint))} \t {range(len(blueprint[0]))} \t {range(len(blueprint[0][0]))}\n\n")
    for i in range(len(blueprint)):

        for j in range(len(blueprint[0])):
            # print(f"\t\tMIDDLE: {j}")
            for k in range(len(blueprint[0][0])):
                # print(f"\t\tINNERMOST: {k}")
                pos = [i,j,k]
                print(f"\t\tITERATION: {pos}")
                print(f"\n\nVAL: {i} \t {j} \t {k}\n\n")

                if blueprint[i][j][k]:
                    print(f"Adding block at {pos}")
                    structure[i, j, k] = Block(pos, True)
                else:
                    print(f"Nothing to add at {pos}")
                    structure[i, j, k] = Block(pos, False)

                # for i in range(len(structure)):
                #     for j in range(len(structure[0])):
                #         for k in range(len(structure[0][0])):
                #             try:
                #                 print(f"Iterating: {structure[i, j, k].position[0]}-{structure[i, j, k].position[1]}-{structure[i, j, k].position[2]}")
                #             except:
                #                 print(f"Iterating: {structure[i, j, k]}")

    # # for block in structure:
    # for i in range(len(structure)):
    #     for j in range(len(structure[0])):
    #         for k in range(len(structure[0][0])):
    #             try:
    #                 print(f"FINAL: {structure[i, j, k].position[0]}-{structure[i, j, k].position[1]}-{structure[i, j,k].position[2]}")
    #             except:
    #                 print(f"FINAL: {structure[i, j, k]}")

    return structure

def spiral_sort(blueprint):
    sorted_blueprint = []

    # blueprint = np.array([[[1, 2, 3], [4, 5, 6]],
    #      [[7, 8, 9], [10, 11, 12]],
    #      [[13, 14, 15], [16, 17, 18]]])

    # print(f"Blueprint: {blueprint}")
    print(f"Blueprint size: {len(blueprint)} -- {len(blueprint[0])} -- {len(blueprint[0][0])}")
    rows = len(blueprint)
    columns = len(blueprint[0])
    for index in range(len(blueprint[0][0])):
        layer = spiral_sort_helper(rows, columns, blueprint[:, :, index])
        layer = layer[::-1]
        sorted_blueprint.append(layer)
        for block in layer:
            print(f"Spiral sorted block at {block.position[0]}-{block.position[1]}-{block.position[2]}")
    print(sorted_blueprint)
    return sorted_blueprint[::-1]



def spiral_sort_helper(m, n, a):
    sorted_array = []
    k = 0
    l = 0

    ''' k - starting row index 
        m - ending row index 
        l - starting column index 
        n - ending column index 
        i - iterator '''

    while (k < m and l < n):

        for i in range(l, n):
            # print(a[k][i], end=" ")
            sorted_array.append(a[k][i])

        k += 1


        for i in range(k, m):
            # print(a[i][n - 1], end=" ")
            sorted_array.append(a[i][n-1])


        n -= 1

        if (k < m):

            for i in range(n - 1, (l - 1), -1):
                # print(a[m - 1][i], end=" ")
                sorted_array.append(a[m-1][i])

            m -= 1

        if (l < n):
            for i in range(m - 1, k - 1, -1):
                # print(a[i][l], end=" ")
                sorted_array.append(a[i][l])

            l += 1
    return sorted_array


class Block:
    def __init__(self, position, hasBlock):
        self.position = position
        self.hasBlock = hasBlock

class Division:
    def __init__(self, x_range, y_range, z_range, status=DivisionStates.UNCLAIMED, owner=None, centroid=None):
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.status = status
        self.owner = owner
        self.centroid = self.calculate_centroid() if centroid is None else centroid
        self.area = self.area()

    def change_status(self, new_status):
        self.status = new_status

    def claim(self, owner):
        self.owner = owner

    def calculate_centroid(self):
        x = (self.x_range[0] + self.x_range[1])/2
        y = (self.y_range[0] + self.y_range[1]) / 2
        z = (self.z_range[0] + self.z_range[1]) / 2
        return (x, y, z)

    def area(self):
        x_start, x_end = self.x_range
        y_start, y_end = self.y_range
        z_start, z_end = self.z_range

        x = x_end-x_start
        y = y_end-y_start
        z = z_end-z_start

        return x*y*z


    def __add__(self, other):
        x = self.x_range + other.x_range
        y = self.y_range + other.y_range
        z = self.z_range + other.z_range

        return Division(x, y, z, self.status, self.owner, self.centroid)

if __name__ == '__main__':
    blueprint = np.array([
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
        [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
        [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
    ])
    blueprint = np.array([
        [[1]*9]*9,
    ]*9)

    # blueprint = np.array([[[1, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 0, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 0, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1]],
    #    [[1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 0, 1, 1, 0, 1, 1, 1],
    #     [1, 1, 1, 1, 1, 1, 1, 1],
    #     [0, 0, 0, 0, 0, 1, 1, 1]]])
    buildingPlanner = BuildingPlanner(blueprint)
    # buildingPlanner.show_structure()
    # spiral_sort(blueprint)
    # buildingPlanner.structure
    divisions = buildingPlanner.create_divisions()
    print(len(divisions))
    buildingPlanner.display()