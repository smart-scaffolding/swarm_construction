from math import pi

from components.simulator.common import transforms as tr
from components.simulator.common.serial_link import SerialLink, Revolute, Prismatic
from .create_actors import *
from .graphics import *


class Inchworm(SerialLink):
    def __init__(self, base=None, blueprint=None, port=None, baud=9600):
        # 4.125 inches
        # 6.429 inches

        seg_lens = np.array([1.04775, 1.632966, 1.632966, 1.04775, 0.049]) * 1.3
        links = [
            Revolute(
                d=seg_lens[0],
                a=0,
                alpha=pi / 2,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[0],
            ),
            Revolute(
                d=0,
                a=seg_lens[1],
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(0, 0),
                length=seg_lens[1],
            ),
            Revolute(
                d=0,
                a=seg_lens[2],
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(-180 * pi / 180, 180 * pi / 180),
                length=seg_lens[2],
            ),
            Revolute(
                d=0,
                a=seg_lens[3],
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(-180 * pi / 180, 180 * pi / 180),
                length=seg_lens[3],
            ),
            Prismatic(
                d=0,
                a=seg_lens[4],
                alpha=0,
                j=0,
                theta=0,
                offset=0,
                qlim=(-180 * pi / 180, 180 * pi / 180),
                length=seg_lens[4],
            ),
        ]

        if base is None:
            base = tr.trotx(-90, unit="deg")
        # else:
        # print(base)
        # assert ishomog(base, (4, 4)) #TODO: May want to renable this
        super(Inchworm, self).__init__(links=links, base=base, name="inchworm")
