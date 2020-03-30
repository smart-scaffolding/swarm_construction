from __future__ import print_function

from abc import ABC
from math import pi

from . import transforms
from .common import *


class SerialLink:
    """
    SerialLink object class.
    """

    def __init__(
        self,
        links,
        name=None,
        base=None,
        tool=None,
        stl_files=None,
        q=None,
        colors=None,
        param=None,
        blueprint=None,
    ):
        """
        Creates a SerialLink object.
        :param links: a list of links that will constitute SerialLink object.
        :param name: name property of the object.
        :param base: base transform applied to the SerialLink object.
        :param stl_files: STL file names to associate with links. Only works for pre-implemented models in model module.
        :param q: initial angles for link joints.
        :param colors: colors of STL files.
        """
        self.pipeline = None
        self.links = links
        if q is None:
            self.q = np.matrix([0 for each in links])
        if base is None:
            self.base = np.asmatrix(np.eye(4, 4))
        else:
            assert (type(base) is np.matrix) and (base.shape == (4, 4))
            self.base = base
        if tool is None:
            self.tool = np.asmatrix(np.eye(4, 4))
        else:
            assert (type(tool) is np.matrix) and (tool.shape == (4, 4))
            self.tool = tool
        # Following arguments initialised by plot function and animate functions only
        # if stl_files is None:
        #     # Default stick figure model code goes here
        #     pass
        # else:
        #     self.stl_files = stl_files
        if name is None:
            self.name = ""
        else:
            self.name = name
        # if colors is None:
        #     self.colors = vtk_named_colors(["Grey"] * len(stl_files))
        # else:
        #     self.colors = colors
        # if param is None:
        #     # If model deosn't pass params, then use these default ones
        #     self.param = {
        #         "cube_axes_x_bounds": np.matrix([[-1.5, 1.5]]),
        #         "cube_axes_y_bounds": np.matrix([[-1.5, 1.5]]),
        #         "cube_axes_z_bounds": np.matrix([[-1.5, 1.5]]),
        #         "floor_position": np.matrix([[0, 0, 0]])
        #     }
        # else:
        #     self.param = param

        # if blueprint is None:
        #     raise Exception("Please provide a blueprint")
        # else:
        #     self.blueprint = blueprint

        self.scale = 0.013

    # time.sleep(2)

    def __iter__(self):
        return (each for each in self.links)

    @property
    def length(self):
        """
        length property
        :return: int
        """
        # return len(self.links)
        return 4

    def fkine(self, stance, unit="rad", apply_stance=False, num_links=4):
        """
        Calculates forward kinematics for a list of joint angles.
        :param stance: stance is list of joint angles.
        :param unit: unit of input angles.
        :param apply_stance: If True, then applied tp actor_list.
        :param actor_list: Passed to apply transformations computed by fkine.
        :param timer: internal use only (for animation).
        :return: homogeneous transformation matrix.
        """

        # flipped=False
        actors = []
        if apply_stance is None:
            raise Exception("Must have data to update with")
        if type(stance) is np.ndarray:
            stance = np.asmatrix(stance)
        if unit == "deg":
            stance = stance * pi / 180
        # if timer is None:
        timer = 0
        # if apply_stance:
        #     self.base = base
        if stance is None:
            t = self.links[0].transform_matrix
        else:
            t = self.base

            angles = stance[timer, 0]

            t = t * self.links[0].A(angles)
        if apply_stance:
            actors.append(transforms.np2vtk(t))
            # actor_list[0].SetUserMatrix(transforms.np2vtk(t))
            # actor_list[0].SetScale(self.scale)
        for i in range(1, num_links, 1):
            # print("I: {}".format(i))
            # print("\tT: {}".format(t))
            if stance is None:
                t = t * self.links[i].transform_matrix
            else:
                angles = stance[timer, i]

                t = t * self.links[i].A(angles)
            if apply_stance:
                actors.append(transforms.np2vtk(t))
                #
                # actor_list[i].SetUserMatrix(transforms.np2vtk(t))
                # actor_list[i].SetScale(self.scale)

        t = t * self.tool

        return t, actors


class Link(ABC):
    """
    Link object class.
    """

    def __init__(
        self,
        j,
        theta,
        d,
        a,
        alpha,
        length,
        offset=None,
        kind="",
        mdh=0,
        flip=None,
        qlim=None,
    ):
        """
        initialises the link object.
        :param j:
        :param theta:
        :param d:
        :param a:
        :param alpha:
        :param offset:
        :param kind: 'r' or 'p' as input. 'r' for Revolute. 'p' for Prismatic.
        :param mdh:
        :param flip:
        :param qlim:
        """
        self.theta = theta
        self.d = d
        # self.j = j
        self.a = a
        self.alpha = alpha
        self.offset = offset
        self.kind = kind
        self.mdh = mdh
        self.flip = flip
        self.qlim = qlim
        self.length = length

        self.max_velocity = 0

        self.set_theta(theta)
        self.velocity = 0  # Link's current velocity

    def set_theta(self, theta):
        """Sets theta to the new theta and computes the new
        transformation matrix

        :param theta: The new theta for the link
        :type theta: int

        :rtype: None
        """
        self.theta = theta
        self.transform_matrix = self.A(theta)

    def update_velocity(self, accel, time):
        """Updates the current velocity of the link when acted upon
        by some acceleration over some time

        :param accel: The acceleration acting upon the link
                      (radians per second^2)
        :type accel: int

        :param time: The time the accelration is applied over (seconds)
        :type time: int

        :rtype: None
        """
        new_velocity = self.velocity + (accel * time)
        if new_velocity <= self.max_velocity:
            self.velocity = new_velocity
            new_theta = self.theta + (new_velocity * time)
            new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
            self.set_theta(new_theta)

    def display(self, unit="rad"):
        """Display the link's properties nicely

        :rtype: None
        """
        angle = self.theta
        if unit == "deg":
            angle = angle * 180 / pi
        print("Link angle: {}".format(angle))
        print("Link length: {}".format(self.length))

    def A(self, q):
        sa = math.sin(self.alpha)
        ca = math.cos(self.alpha)
        if self.flip:
            q = -q + self.offset
        else:
            q = q + self.offset
        st = 0
        ct = 0
        d = 0
        if self.kind == "r":
            st = math.sin(q)
            ct = math.cos(q)
            d = self.d
        elif self.kind == "p":
            st = math.sin(self.theta)
            ct = math.cos(self.theta)
            d = q

        se3_np = 0
        if self.mdh == 0:
            se3_np = np.matrix(
                [
                    [ct, -st * ca, st * sa, self.a * ct],
                    [st, ct * ca, -ct * sa, self.a * st],
                    [0, sa, ca, d],
                    [0, 0, 0, 1],
                ]
            )

        return se3_np


class Revolute(Link):
    """
    Revolute object class.
    """

    def __init__(self, j, theta, d, a, alpha, offset, qlim, length):
        """
        Initialised revolute object.
        :param j:
        :param theta:
        :param d:
        :param a:
        :param alpha:
        :param offset:
        :param qlim:
        """
        super().__init__(
            j=j,
            theta=theta,
            d=d,
            a=a,
            alpha=alpha,
            offset=offset,
            kind="r",
            qlim=qlim,
            length=length,
        )
        pass


class Prismatic(Link):
    """
    Prismatic object class.
    """

    def __init__(self, j, theta, d, a, alpha, offset, qlim, length):
        """
        Initialises prismatic object.
        :param j:
        :param theta:
        :param d:
        :param a:
        :param alpha:
        :param offset:
        :param qlim:
        """
        super().__init__(
            j=j,
            theta=theta,
            d=d,
            a=a,
            alpha=alpha,
            offset=offset,
            kind="p",
            qlim=qlim,
            length=length,
        )
        pass

    pass
