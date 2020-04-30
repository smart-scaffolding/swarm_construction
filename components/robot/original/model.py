from math import pi

import numpy as np

from components.robot.original import transforms as tr
from components.robot.original.common import ishomog
from components.robot.original.serial_link import SerialLink, Revolute


class Inchworm(SerialLink):
    def __init__(self, base=None, blueprint=None, port=None, baud=9600):

        # self.qn = np.matrix([[0, pi / 4, pi, 0, pi / 4, 0]])
        # self.qr = np.matrix([[0, pi / 2, -pi / 2, 0, 0, 0]])
        # self.qz = np.matrix([[0, 0, 0, 0, 0, 0]])
        # self.qs = np.matrix([[0, 0, -pi / 2, 0, 0, 0]])
        # self.scale = 0.1
        param = {
            "cube_axes_x_bounds": np.matrix([[0, len(blueprint)]]),
            "cube_axes_y_bounds": np.matrix([[0, len(blueprint[0])]]),
            "cube_axes_z_bounds": np.matrix([[0, len(blueprint[0][0])]]),
            "floor_position": np.matrix([[0, 0, 0]]),
        }

        # 4.125 inches A link
        # 6.429 inches B link

        # seg_lens = np.array([1.04775, 1.632966, 1.632966, 1.04775])*1.3
        # seg_lens = np.array([1.375, 2.143, 2.143, 1.375])
        seg_lens = np.array([0.489204, 0.558546, 1.63322, 1.63322, 0.558546, 0.489204]) * 1.3

        # Original model
        links = [
            # Revolute(
            #     d=seg_lens[0],
            #     a=0,
            #     alpha=pi / 2,
            #     j=0,
            #     theta=0,
            #     offset=0,
            #     qlim=(0, 0),
            #     length=seg_lens[0],
            # ),
            # Revolute(
            #     d=0,
            #     a=seg_lens[1],
            #     alpha=0,
            #     j=0,
            #     theta=0,
            #     offset=0,
            #     qlim=(0, 0),
            #     length=seg_lens[1],
            # ),
            # Revolute(
            #     d=0,
            #     a=seg_lens[2],
            #     alpha=0,
            #     j=0,
            #     theta=0,
            #     offset=0,
            #     qlim=(-180 * pi / 180, 180 * pi / 180),
            #     length=seg_lens[2],
            # ),
            # Revolute(
            #     d=0,
            #     a=seg_lens[3],
            #     alpha=0,
            #     j=0,
            #     theta=0,
            #     offset=0,
            #     qlim=(-180 * pi / 180, 180 * pi / 180),
            #     length=seg_lens[3],
            # ),
            # 1 
            Revolute(
                d=seg_lens[0],
                a=0,
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[0],
            ),
            # 1.5
            Revolute(
                d=seg_lens[1],
                a=0,
                alpha=pi / 2,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[1],
            ),
            # 2
            Revolute(
                d=0,
                a=0,
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=0,
            ),
            # 3
            Revolute(
                d=0,
                a=seg_lens[2],
                alpha=0,
                j=0,
                theta=pi / 2,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[2],
            ),
            # 4
            Revolute(
                d=0,
                a=seg_lens[3],
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[3],
            ),

            # 4.5
            Revolute(
                d=0,
                a=seg_lens[4],
                alpha=0,
                j=0,
                theta=pi / 2,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[4],
            ),
            # 5
            Revolute(
                d=0,
                a=0,
                alpha=pi / 2,
                j=0,
                theta=pi / 2,
                offset=0,
                qlim=(0, 0),
                length=0,
            ),
            # 6
            Revolute(
                d=seg_lens[5],
                a=0,
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[5],
            ),
        ]

        if base is None:
            base = tr.trotx(-90, unit="deg")
        else:
            assert ishomog(base, (4, 4))
        file_names = SerialLink._setup_file_names(4)
        # colors = graphics.vtk_named_colors(["Red", "DarkGreen", "Blue", "Cyan"])

        super().__init__(
            links=links,
            base=base,
            name="inchworm",
            stl_files=file_names,
            colors=None,
            param=param,
            blueprint=blueprint,
            port=port,
            baud=baud,
        )
