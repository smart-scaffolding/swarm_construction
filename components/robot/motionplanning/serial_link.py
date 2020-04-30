from __future__ import print_function

import time
from abc import ABC
from math import pi, cos, sin, atan2, acos, sqrt

import numpy as np
from scipy.spatial.transform import Rotation as R
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS

import components.robot.motionplanning.transforms as transforms
from components.robot.motionplanning.common import *


class SerialLink:
    """
    SerialLink object class.
    """

    def __init__(
        self,
        links,
        a_link_starting_pos,
        d_link_starting_pos,
        name=None,
        tool=None,
        stl_files=None,
        q=None,
        colors=None,
        param=None,
        blueprint=None,
        port=None,
        baud=9600,
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
        # if base is None:
        #     self.base = np.asmatrix(np.eye(4, 4))
        # else:
        #     # assert (type(base) is np.matrix) and (base.shape == (4, 4))
        #     self.base = base
        if tool is None:
            self.tool = np.asmatrix(np.eye(4, 4))
        else:
            assert (type(tool) is np.matrix) and (tool.shape == (4, 4))
            self.tool = tool
        # Following arguments initialised by plot function and animate functions only
        if stl_files is None:
            # Default stick figure model code goes here
            pass
        else:
            self.stl_files = stl_files
        if name is None:
            self.name = ""
        else:
            self.name = name

        self.colors = colors
        if param is None:
            # If model deosn't pass params, then use these default ones
            self.param = {
                "cube_axes_x_bounds": np.matrix([[-1.5, 1.5]]),
                "cube_axes_y_bounds": np.matrix([[-1.5, 1.5]]),
                "cube_axes_z_bounds": np.matrix([[-1.5, 1.5]]),
                "floor_position": np.matrix([[0, 0, 0]]),
            }
        else:
            self.param = param

        if blueprint is None:
            raise Exception("Please provide a blueprint")
        else:
            self.blueprint = blueprint

        self.scale = 0.013
        if port is not None:
            self.serial = Serial(
                port=port,
                baudrate=baud,
                parity=PARITY_NONE,
                stopbits=STOPBITS_ONE,
                bytesize=EIGHTBITS,
                timeout=3.0,
            )
        else:
            self.serial = None

        self.DEBUG = False
        self.last_placed_block = False
        # Robot state variables
        self.AEE_POSE = None
        self.DEE_POSE = None
        self.primary_ee = "D"
        self.update_angles(
            np.array([1.61095456e-15, 6.18966422e01, -1.23793284e02, -2.80564688e01])
        )
        self.resetEEStartingPoses(a_link_starting_pos, d_link_starting_pos)

    time.sleep(2)

    def __iter__(self):
        return (each for each in self.links)

    def reset(self):
        """Resets the arm back to its resting state, i.e. q0

        :rtype: None
        """
        self.update_angles(self.q)

    def get_current_joint_config(self, unit="rad"):
        """Gets the current joint configuration from the links

        :returns: 1xN vector of current joint config
        :rtype: numpy.ndarray
        """
        q = np.zeros(self.length)
        for i, link in enumerate(self.links):
            q[i] = link.theta
        if unit == "deg":
            q = q * 180 / pi
        return q

    def update_angles(self, new_angles, save=False, unit="rad"):
        """Updates all the link's angles

        :param new_angles: 1xN vector of new link angles
        :type new_angles: numpy.ndarray

        :param save: Flag that determines if the update is cached
        :param save: bool

        :rtype: None
        """

        # THIS EXPECTS EVERYTHING TO BE IN DEGREES
        # SETS LINK ANGLES TO RADIANS
        for link, new_theta in zip(self.links, new_angles):
            if unit == "deg":
                new_theta = new_theta * np.pi / 180
            link.set_theta(new_theta)

    def update_link_angle(self, link, new_angle, save=False):
        """Updates the given link's angle with the given angle

        :param link: The link you want to update
        :type link: int

        :param new_angle: The link's new angle
        :type new_angle: int

        :param save: Flag that determines if the update is cached
        :type save: bool

        :rtype: None
        """
        self.links[link].set_theta(new_angle)

    # TODO: Acceleration over time seems like a weird way to update this
    def update_link_velocity(self, link, accel, time):
        """Updates the given link's velocity with the given
        acceleration over the given time

        :param link: The link you want to update
        :type link: int

        :param accel: The acceleration (Radians per second^2)
        :type accel: int

        :param time: The time (Seconds)
        :type time: int

        :rtype: None
        """
        self.links[link].update_velocity(accel, time)
        # self.update_link_positions()

    def end_effector_position(self, q=None, transform=False, num_links=None):
        """Return end effector position

        :param q: Config to compute the end effector position for a given
                  1xN q vector
        :type q: numpy.ndarray or None

        :returns: Position (x, y, z) of end effector
        :rtype: numpy.ndarray
        """
        if q is None:
            q = self.get_current_joint_config()

        if num_links is None:
            num_links = self.length

        t = self.fkine(stance=q, num_links=num_links)
        if transform:
            return t
        end_pos = create_point_from_homogeneous_transform(t).T
        return end_pos

    def end_effector_velocity(self):
        """Calculate the end effector velocity of the arm given
        its current angular velocities.

        :returns: Returns linear and angular velocity in each dimension
                  (vx, vy, vz, wx, wy, wz).
        :rtype: np.ndarray
        """
        q = np.array([link.theta for link in self.links])
        dq = np.array([link.velocity for link in self.links])

        velocity = self.jacob0(q) * np.asmatrix(dq).T
        return velocity.A1

    @property
    def length(self):
        """
        length property
        :return: int
        """
        # return len(self.links)
        return 4

    def get_velocity_controller_term(self, index, total_num_points, offset):
        """

        :param index:
        :param total_num_points:
        :param offset:
        :return:
        """
        lower_vel_bound = (total_num_points / 3) - offset
        upper_vel_bound = total_num_points / 3
        num_via_points = total_num_points / 3
        if (
            index % num_via_points >= lower_vel_bound
            and index % num_via_points <= upper_vel_bound
        ):
            return 0
        return 1

    def send_to_robot(
        self,
        angle,
        index,
        total_num_points,
        velocity_offset=2,
        delay=2.0,
        open_gripper="00",
    ):
        """
        NOTE: Expects all angles to be in degrees
        Sends a single angle to robot and then delays for a certain amount of time

        :param angle: Expects angles in degrees
        :param delay: delay after sending to robot
        """
        velocity_controller_term = self.get_velocity_controller_term(
            index, total_num_points, velocity_offset
        )
        targetAngles = self.map_angles_to_robot(
            angle, open_gripper, velocity_controller_term
        )
        self.serial.write(targetAngles)
        time.sleep(delay)

    def map_angles_to_robot(self, q, open_gripper="00", velocity_controller_term=0):
        """
        Creates a mapping between the angles used by the higher level code and the actual robot angles

        :param q: Input angle, expects angles in degrees
        :return:
        """

        qTemp = np.array([q[0], 90 - q[1], q[2] * -1, q[3] * -1])
        # qTemp = qTemp * 180.0 / np.pi  # convert to degrees
        print("Final Angles: {}".format(qTemp[1:]))

        # gripper = "0002"
        # if open_gripper:
        gripper = str(velocity_controller_term) + "0" + open_gripper

        targetAngles = (
            f"{qTemp[1]:4.2f} ".zfill(8)
            + f"{qTemp[2]:4.2f} ".zfill(8)
            + f"{qTemp[3]:4.2f} ".zfill(8)
            + gripper
            + "\n"
        )
        print(targetAngles)
        return str.encode(targetAngles)

    def gripper_control_commands(
        self, engage_gripper, disengage_gripper, flip_pid, toggle_gripper
    ):
        """

        :param engage_gripper:
        :param disengage_gripper:
        :param flip_pid:
        :param toggle_gripper:
        :return:
        """
        if engage_gripper:
            gripper_control = "0"
        elif disengage_gripper:
            gripper_control = "1"
        else:
            gripper_control = "2"  # stop gripper (idle)

        pid = "1" if flip_pid else "0"
        select_gripper = "0"
        if toggle_gripper:
            if select_gripper == "0":
                select_gripper = "1"
            else:
                select_gripper = "0"

        return "0" + pid + select_gripper + gripper_control

    def fkine(
        self,
        stance,
        unit="rad",
        apply_stance=False,
        actor_list=None,
        timer=None,
        num_steps=0,
        num_links=4,
        directions=None,
        orientation=None,
        update=None,
    ):
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
        has_rotation = [1, 3, 4, 5, 7]

        if apply_stance and update is None:
            raise Exception("Must have data to update with")
        if type(stance) is np.ndarray:
            stance = np.asmatrix(stance)
        if unit == "deg":
            stance = stance * pi / 180
        if timer is None:
            timer = 0

            angles = stance[timer, 0]

            # t = t * self.links[0].A(angles)
            t = self.links[0].transform_matrix
        else:
            t = self.base

        for i in range(1, 8, 1):
            if i not in has_rotation:
                t = t * self.links[i].transform_matrix
            else:
                angles = stance[timer, i]

                t = t * self.links[i].A(angles)
            if apply_stance:
                actor_list[i].SetUserMatrix(transforms.np2vtk(t))
                actor_list[i].SetScale(self.scale)

        t = t * self.tool
        return t

    # Units: inches, radians
    # inputs should be in the global reference frame

    def ikin(
        self, goalPos, gamma, phi, baseID, simHuh=False, elbow_up=1, placeBlock=False
    ):
        """

        :param goalPos:
        :param gamma:
        :param phi:
        :param baseID:
        :param simHuh:
        :param elbow_up:
        :param placeBlock:
        :return:
        """
        if self.DEBUG:
            print(f"(Ikin)goalPos:{goalPos} Gamma:{gamma} Phi:{phi} baseID:{baseID}")
        # Robot Parameters
        # L1 = 4.125  # L1 in inches
        # L2 = 6.43  # L2 in inches
        # blockWidth = 3
        # L1 = self.links[0].length * 10
        # L2 = self.links[1].length
        L1 = self.links[0].length + self.links[1].length
        L2 = self.links[3].length

        relativePos, localGamma = self.handlePlaneChanges(
            goalPos=goalPos, gamma=gamma, baseID=baseID, placeBlock=placeBlock
        )
        # x, y, z, dummy = relativePos * blockWidth  # dummy value should always 1
        x, y, z, dummy = relativePos  # dummy value should always 1

        if self.DEBUG:
            print(f"x y z: {x} {y} {z}")
            print(f"gamma: {gamma}")
            print(f"localGamma: {localGamma}")
            print(f"relativePos: {relativePos}")

        q1 = atan2(y, x)  # joint1 angle
        # if q1 < -179.8:
        #     q1 = 0

        new_z = z - L1  # take away the height of the first link (vertical)
        # new_x = x / cos(q1)
        new_x = sqrt(x ** 2 + y ** 2)

        x3 = new_x - L1 * cos(localGamma)
        z3 = new_z - L1 * sin(localGamma)  # reduce to the 2dof planar robot

        # print(f"Square: {x3 ** 2 + z3 ** 2}")
        # print(f"ACos: {(L2 ** 2 + (x3 ** 2 + z3 ** 2) - L2 ** 2) / (2 * L2 * sqrt(x3 ** 2 + z3 ** 2))}")
        beta = acos(
            (L2 ** 2 + (x3 ** 2 + z3 ** 2) - L2 ** 2) / (2 * L2 * sqrt(x3 ** 2 + z3 ** 2))
        )

        if elbow_up == 1:
            q3 = 2 * beta
            q2 = pi / 2 - (atan2(z3, x3) + beta)

        elif elbow_up == 0:
            q3 = -2 * beta
            q2 = pi / 2 - (atan2(z3, x3) - beta)

        q4 = (localGamma - pi / 2 + q2 + q3) * -1
        q5 = phi + q1
        q = np.array([q1, q2, q3, q4, q5])

        # check which ee is requested and flip angles accordingly
        if not simHuh:
            if baseID == "D":
                q = np.array([q5, q4, q3, q2, q1])

        if self.DEBUG:
            print(f"ikin output q: {q}\n\n")
        return q

    def handlePlaneChanges(self, goalPos, gamma, baseID, placeBlock=False):
        """

        :param goalPos:
        :param gamma:
        :param baseID:
        :param placeBlock:
        :return:
        """
        if self.DEBUG:
            print(f"Pre plane handling:\n AEE_POSE: {self.AEE_POSE}")
            print(f"DEE POS: {self.DEE_POSE}")

        relativePos = np.zeros(3)
        baseOri = np.zeros(3)
        basePos = np.zeros(3)
        goalRot = np.zeros(3)

        # gets the relativePos, basePos, and baseOri in the global reference frame
        if baseID == "A":  # requested base is A
            basePos = self.AEE_POSE[:3, 3].T
            baseOri = R.from_matrix(self.AEE_POSE[:3, :3]).as_euler("xyz", degrees=True)
        elif baseID == "D":  # requested base is D
            basePos = self.DEE_POSE[:3, 3].T
            baseOri = R.from_matrix(self.DEE_POSE[:3, :3]).as_euler("xyz", degrees=True)

        relativePos = np.append(goalPos - basePos, [1])

        # print(f'AEE_POSE {self.AEE_POSE}')
        # print(f'DEE_POSE {self.DEE_POSE}')
        # print(f'goalPos {goalPos}')
        # print(f'basePos {basePos}')
        # print(f'relativePos {relativePos}')

        # the angle which the arm is extending towards from base ee
        # in global reference frame
        # !! This needs to be done before switching relativePos into local frame
        armFacing = atan2(relativePos[1], relativePos[0])
        if self.DEBUG:
            print(f"relativePos: {relativePos}")
            print(f"armFacing: {armFacing}")

        # rotates relativePos from global reference frame to local reference frame
        if baseOri[0] > 0.1:  # local +z facing global +x, rotate -90 around y
            relativePos = np.dot(self.getRy(-pi / 2), relativePos)
        elif baseOri[0] < -0.1:  # local +z facing global -x, rotate 90 around y
            relativePos = np.dot(self.getRy(pi / 2), relativePos)
        elif baseOri[1] > 0.1:  # local +z facing global +y, rotate 90 around x
            relativePos = np.dot(self.getRx(pi / 2), relativePos)
        elif baseOri[1] < -0.1:  # local +z facing global -y, rotate -90 around x
            relativePos = np.dot(self.getRx(-pi / 2), relativePos)
        elif baseOri[2] > 0.1:  # local +z facing global +z, do nothing
            pass
        elif baseOri[2] < -0.1:  # local +z facing global -z, rotate 180 around y
            relativePos = np.dot(self.getRy(pi), relativePos)

        if self.DEBUG:
            print(f"relativePos in local frame: {relativePos}")
        localGamma = gamma

        # updates goal orientation and
        # switches gamma value from global reference frame to local reference frame
        if gamma == -pi / 2:  # goal ee will be facing down
            # goalRot = np.array([0,0,1,0]).T
            goalRot = self.getRz(pi / 2)
            if baseOri[2] > 0.1:  # base ee down
                pass
            elif baseOri[2] < -0.1:  # base ee up
                localGamma = -gamma
            else:  # base ee horizontal
                localGamma = 0
        elif gamma == pi / 2:  # goal ee will be facing up
            # goalRot = np.array([0,0,-1,0]).T
            goalRot = self.getRz(-pi / 2)
            if baseOri[2] > 0.1:  # base ee down
                pass
            elif baseOri[2] < -0.1:  # base ee up
                localGamma = -gamma
            else:  # base ee horizontal
                localGamma = 0
        elif gamma == 0 or gamma == -pi:  # goal ee horizontal
            if armFacing == 0 or armFacing == pi:  # goal ee facing right (positive X)
                if gamma == 0:
                    # goalRot = np.array([-1,0,0,0]).T
                    goalRot = self.getRx(-pi / 2)
                else:
                    # goalRot = np.array([1,0,0,0]).T
                    goalRot = self.getRx(pi / 2)
            elif armFacing == -pi:  # goal ee facing left (negative X)
                if gamma == 0:
                    # goalRot = np.array([1,0,0,0]).T
                    goalRot = self.getRx(pi / 2)
                else:
                    # goalRot = np.array([-1,0,0,0]).T
                    goalRot = self.getRx(-pi / 2)
            elif armFacing == pi / 2:  # goal ee facing back (positive Y)
                if gamma == 0:
                    # goalRot = np.array([0,-1,0,0]).T
                    goalRot = self.getRy(-pi / 2)
                else:
                    # goalRot = np.array([0,1,0,0]).T
                    goalRot = self.getRy(pi / 2)
            elif armFacing == -pi / 2:  # goal ee facing front (negative Y)
                if gamma == 0:
                    # goalRot = np.array([0,1,0,0]).T
                    goalRot = self.getRy(pi / 2)
                else:
                    # goalRot = np.array([0,-1,0,0]).T
                    goalRot = self.getRy(-pi / 2)

            if baseOri[2] > 0.1 or baseOri[2] < -0.1:  # base ee down or up
                pass
            else:  # base ee horizontal
                # TODO: original if using all() would work
                # if baseOri.all() != goalRot.all():
                if baseOri.all() != R.from_matrix(goalRot[:3]).as_euler(
                    "xyz", degrees=True
                ):
                    localGamma = pi / 2
                else:
                    localGamma = -pi / 2

        # sets the goalPos and goalOri to the moving ee

        # if self.last_placed_block:
        #     goalPos_list[2] = goalPos_list[2] - 1
        #     goalPos = tuple(goalPos_list)
        #     self.last_placed_block = False
        #
        # if placeBlock:
        #     # goalPos_list[2] = goalPos_list[2] - 1
        #     # goalPos = tuple(goalPos_list)
        #     self.last_placed_block = True

        if baseID == "A":  # requested ee is A, update D to match goal
            # DEEPOS = goalPos
            # DEEORI = goalOri
            self.DEE_POSE[:3, 3] = np.array(goalPos)
            self.DEE_POSE[:3, :3] = goalRot[:3, :3]
        elif baseID == "D":  # requested ee is D, update A to match goal
            # AEEPOS = goalPos
            # AEEORI = goalOri
            self.AEE_POSE[:3, 3] = np.array(goalPos)
            self.AEE_POSE[:3, :3] = goalRot[:3, :3]

        if self.DEBUG:
            print(f"Post plane handling:\n AEE_POSE: {self.AEE_POSE}")
            print(f"DEE POS: {self.DEE_POSE}")
        return relativePos, localGamma

    def getRx(self, theta):
        """

        :param theta:
        :return:
        """
        T = np.eye(4)
        T[1, :] = [0, cos(theta), -sin(theta), 0]
        T[2, :] = [0, sin(theta), cos(theta), 0]
        return T

    def getRy(self, theta):
        """

        :param theta:
        :return:
        """
        T = np.eye(4)
        T[0, :] = [cos(theta), 0, sin(theta), 0]
        T[2, :] = [-sin(theta), 0, cos(theta), 0]
        return T

    def getRz(self, theta):
        """

        :param theta:
        :return:
        """
        T = np.eye(4)
        T[0, :] = [cos(theta), -sin(theta), 0, 0]
        T[1, :] = [sin(theta), cos(theta), 0, 0]
        return T

    def resetEEStartingPoses(self, a_link_starting_pos, d_link_starting_pos):
        """

        :param a_link_starting_pos:
        :param d_link_starting_pos:
        """
        # self.AEEPOS = np.array([0,0,0]).T
        # self.AEEORI = np.array([0,0,1]).T
        # self.DEEPOS = np.array([2,0,0]).T
        # self.DEEORI = np.array([0,0,1]).T
        self.AEE_POSE = np.matmul(np.array(np.eye(4)), self.getRz(np.pi / 2))
        self.DEE_POSE = np.matmul(np.array(np.eye(4)), self.getRz(np.pi / 2))
        # x y z position
        self.AEE_POSE[:3, 3] = np.array(a_link_starting_pos)
        self.DEE_POSE[:3, 3] = np.array(d_link_starting_pos)

    def jacob0(self, q=None):
        """Calculates the jacobian in the world frame by finding it in
        the tool frame and then converting to the world frame.

        :param q: (Optional) 1xN joint configuration to compute the jacobian on
        :type q: numpy.ndarray

        :returns: 6xN Jacobian in the world frame
        :rtype: numpy.matrix
        """

        # Get the tool frame jacobian
        J = self.jacobn(q)

        # Set up homogeneous transform matrix for the world
        eet = self.fkine(q)
        rotation = get_rotation_from_homogeneous_transform(eet)
        zeros = np.zeros((3, 3))
        a1 = np.hstack((rotation, zeros))
        a2 = np.hstack((zeros, rotation))

        # Convert to world frame
        J = np.vstack((a1, a2)) * J
        return J

    def jacobn(self, q=None):
        """Calculates the jacobian in the tool frame

        :param q: (Optional) 1xN joint configuration to compute the jacobian on
        :type q: 1xN numpy.ndarray

        :returns: 6xN Jacobian in the tool frame
        :rtype: numpy.matrix
        """
        J = np.zeros((6, self.length))
        U = self.tool
        I_term = range(self.length - 1, -1, -1)

        for i, link in zip(I_term, self.links[::-1]):
            if np.any(q):
                U = link.A(q[i]) * U
            else:
                U = link.transform_matrix * U

            d = np.array(
                [
                    -U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
                    -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
                    -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3],
                ]
            )
            delta = U[2, 0:3]

            J[:, i] = np.vstack((d, delta)).flatten()
        return J

    # def ikine(self, T, q0=None, unit='rad'):
    #     """
    #     Calculates inverse kinematics for homogeneous transformation matrix using numerical optimisation method.
    #     :param T: homogeneous transformation matrix.
    #     :param q0: initial list of joint angles for optimisation.
    #     :param unit: preferred unit for returned joint angles. Allowed values: 'rad' or 'deg'.
    #     :return: a list of 6 joint angles.
    #     """
    #     assert T.shape == (4, 4)
    #     bounds = [(link.qlim[0], link.qlim[1]) for link in self]
    #     reach = 0
    #     for link in self:
    #         reach += abs(link.a) + abs(link.d)
    #     omega = np.diag([1, 1, 1, 3 / reach])
    #     if q0 is None:
    #         q0 = np.asmatrix(np.zeros((1, self.length)))
    #
    #     def objective(x):
    #         return (
    #             np.square(((np.linalg.lstsq(T, self.fkine(x))[0]) - np.asmatrix(np.eye(4, 4))) * omega)).sum()
    #
    #     sol = minimize(objective, x0=q0, bounds=bounds)
    #     if unit == 'deg':
    #         return np.asmatrix(sol.x * 180 / pi)
    #     else:
    #         return np.asmatrix(sol.x)


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
        """

        :param q:
        :return:
        """
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
