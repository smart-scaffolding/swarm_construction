from components.simulator.common.serial_link import SerialLink
from components.simulator.common.serial_link import Revolute, Prismatic
from math import pi
from components.simulator.common import transforms as tr
from .graphics import *
from .create_actors import *
from components.simulator.common.common import ishomog

# from simulator.model.graphics import *




class Inchworm(SerialLink):
    def __init__(self, base=None, blueprint=None, port=None, baud=9600):

        # self.qn = np.matrix([[0, pi / 4, pi, 0, pi / 4, 0]])
        # self.qr = np.matrix([[0, pi / 2, -pi / 2, 0, 0, 0]])
        # self.qz = np.matrix([[0, 0, 0, 0, 0, 0]])
        # self.qs = np.matrix([[0, 0, -pi / 2, 0, 0, 0]])
        # self.scale = 0.1
        # param = {
        #     "cube_axes_x_bounds": np.array([[0, len(blueprint)]]),
        #     "cube_axes_y_bounds": np.array([[0, len(blueprint[0])]]),
        #     "cube_axes_z_bounds": np.array([[0, len(blueprint[0][0])]]),
        #     "floor_position": np.array([[0, 0, 0]])
        # }

#4.125 inches
#6.429 inches

        seg_lens = np.array([1.04775, 1.632966, 1.632966, 1.04775, 0.049])*1.3
        # seg_lens = np.array([1.375, 2.143, 1.375, 2.143])

        links = [Revolute(d=seg_lens[0], a=0, alpha=pi/2, j=0, theta=0, offset=0, qlim=(0, 0), length=seg_lens[0]),
                 Revolute(d=0, a=seg_lens[1], alpha=0, j=0, theta=0, offset=0, qlim=(0, 0), length=seg_lens[1]),
                 Revolute(d=0, a=seg_lens[2], alpha=0, j=0, theta=0, offset=0, qlim=(-180 * pi / 180, 180 * pi / 180), length=seg_lens[2]),
                 Revolute(d=0, a=seg_lens[3], alpha=0, j=0, theta=0, offset=0, qlim=(-180 * pi / 180, 180 * pi / 180), length=seg_lens[3]),
                 Prismatic(d=0, a=seg_lens[4], alpha=0, j=0, theta=0, offset=0, qlim=(-180 * pi / 180, 180 * pi / 180),
                          length=seg_lens[4]),
                 ]

        if base is None:
            base = tr.trotx(-90, unit='deg')
        # else:
            # print(base)
            # assert ishomog(base, (4, 4)) #TODO: May want to renable this

        file_names = setup_file_names(4)
        # colors = graphics.vtk_named_colors(["Red", "DarkGreen", "Blue", "Cyan"])
        colors = vtk_named_colors(["Red", "Blue", "Blue", "Purple"])

        super(Inchworm, self).__init__(links=links, base=base, name='inchworm')

        # links, name = None, base = None, tool = None, stl_files = None, q = None, colors = None, param = None,
        # blueprint = None