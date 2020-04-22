from __future__ import print_function

import time
from abc import ABC
from math import pi, cos, sin, atan2, acos, sqrt

import numpy as np
import pkg_resources
import vtk
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS

# import robopy.base.transforms as tr
import components.robot.original.transforms as transforms
from components.robot.original.common import *


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
        # Robot state variables
        self.AEE_POSE = None
        self.DEE_POSE = None
        self.resetEEStartingPoses()

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
        # self.update_link_positions()

        # if save:
        #     q = np.array([l.theta for l in self.links])
        #     self.qs = np.vstack((self.qs, q.copy()))

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
        # self.update_link_positions()

        # Save each config for replay
        # if save:
        #     q = np.array([l.theta for l in self.links])
        #     self.qs = np.vstack((self.qs, q.copy()))

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

    # def update_link_positions(self):
    #     """Walk through all the links and update their positions.
    #
    #     :rtype: None
    #     """
    #
    #     for i, link in enumerate(self.links):
    #         # Set link base position
    #         if i == 0:
    #             link.base_pos = utils.create_point_from_homogeneous_transform(
    #                 self.base)
    #         else:
    #             link.base_pos = self.links[i - 1].end_pos
    #
    #         # Set link end position
    #         if link.length == 0 and link.offset == 0:
    #             link.end_pos = link.base_pos
    #         else:
    #             # Compute FKine up to that link endpoint
    #             # to get the location in homogenous coords
    #             t = self.fkine(links=range(i + 1))
    #             # Then convert that to world space
    #             end_pos = utils.create_point_from_homogeneous_transform(t).T
    #             link.end_pos = end_pos.A1

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
        flip_angles=False,
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
            angle, open_gripper, velocity_controller_term, flip_angles=flip_angles
        )
        self.serial.write(targetAngles)
        time.sleep(delay)

    def map_angles_to_robot(
        self, q, open_gripper="00", velocity_controller_term=0, flip_angles=False
    ):
        """
        Creates a mapping between the angles used by the higher level code and the actual robot angles

        :param q: Input angle, expects angles in degrees
        :return:
        """
        if not flip_angles:
            print("NOPE")
            qTemp = np.array([q[0], 90 - q[1], q[2] * -1, q[3] * -1])
        else:
            print("FLIPPING ANGLES")
            qTemp = np.array([q[0], q[3] * -1, q[2] * -1, 90 - q[1]])
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

        if apply_stance and update is None:
            raise Exception("Must have data to update with")
        if type(stance) is np.ndarray:
            stance = np.asmatrix(stance)
        if unit == "deg":
            stance = stance * pi / 180
        if timer is None:
            timer = 0
        # if num_steps > 0 and timer % num_steps == 0:
        #         #     # if timer - num_steps == 0:
        #         #     ee_pos = round_end_effector_position(self.end_effector_position().tolist()[0], "top", None)
        #         #
        #         #
        #         #     index = (timer/num_steps)
        #         #     if index % 2 == 0:
        #         #
        #         #         new_base = flip_base(ee_pos, orientation[int(index)], 0, animation=True)
        #         #
        #         #         flipped=False
        #         #     else:
        #         #         new_base = flip_base(ee_pos, orientation[int(index)], 180, animation=True)
        #         #
        #         #         flipped=True
        #         #
        #         #     # print("EE_POS: {}".format(ee_pos))
        #         #     self.base = new_base

        if apply_stance:
            self.base = update.robot_base
        if stance is None:
            t = self.links[0].transform_matrix
        else:
            t = self.base

            angles = stance[timer, 0]

            t = t * self.links[0].A(angles)
        if apply_stance:
            actor_list[0].SetUserMatrix(transforms.np2vtk(t))
            actor_list[0].SetScale(self.scale)
        for i in range(1, num_links, 1):
            # print("I: {}".format(i))
            # print("\tT: {}".format(t))
            if stance is None:
                t = t * self.links[i].transform_matrix
            else:
                angles = stance[timer, i]

                t = t * self.links[i].A(angles)
            if apply_stance:
                actor_list[i].SetUserMatrix(transforms.np2vtk(t))
                actor_list[i].SetScale(self.scale)

        t = t * self.tool
        return t

    def ikineConstrained(
        self,
        direction,
        p,
        theta=-pi / 2,
        phi=0,
        accuracy=1e-7,
        num_iterations=1000,
        alpha=0.1,
        prior_q=None,
        vertical=False,
        flipped=False,
    ):
        """Computes the inverse kinematics to find the correct joint
        configuration to reach a given point

        :param p: The point (x, y, z) to solve the inverse kinematics for
        :type p: numpy.ndarray

        :param num_iterations: The number of iterations to try before
                               giving up
        :type num_iterations: int

        :param alpha: The stepsize for the ikine solver (0.0 - 1.0)
        :type alpha: int

        :returns: 1xN vector of the joint configuration for given point p.
        :rtype: numpy.ndarray
        """
        # Check to make sure alpha is between 0 and 1
        if not (0.0 <= alpha <= 1.0):
            print("Invalid alpha. Defaulting to 0.1")
            alpha = 0.1
        q = self.get_current_joint_config()
        # q = self.get_current_joint_config()*np.pi/180
        print("\t\tCURRENT JOINT CONFIG: {}".format(q))
        print("\t\tGOING TO POINT IK: {}".format(p))
        print("\t\tEE POS: {}".format(self.end_effector_position()))
        print("\t\tDIRECTION: {}\n".format(direction))
        if flipped:
            temp = q[1]
            q[1] = np.pi / 2 + q[3]
            q[3] = temp - np.pi / 2
        pTarget = np.copy(p)
        # TODO fix the hardcoded value
        pTarget[2] = pTarget[2] + (1.04775 * 1.3)

        goal = create_homogeneous_transform_from_point(pTarget)
        for i in range(num_iterations):
            # Calculate position error of the end effector
            curr = self.fkine(q, num_links=self.length - 1)
            # print("Current EE Pos: {}".format(curr[0:3, 3]))
            # self.update_angles(q)

            err = goal - curr

            # Convert error from homogeneous to xyz space
            err = create_point_from_homogeneous_transform(err)

            # Get the psudoinverse of the Jacobian
            J = self.jacob0(q)
            vel_J = J[0:3, :]

            # Increment q a tiny bit
            delta_q = np.linalg.pinv(vel_J) * err
            delta_q = np.squeeze(np.asarray(delta_q))
            q = q + (alpha * delta_q.flatten())

            if direction == "top":
                q[-1] = q[1] - np.pi / 2
            if direction == "bottom":
                q[-1] = q[-1] + pi

            if direction == "left" or direction == "front":
                q[-1] = q[-1] - np.pi / 2

            if direction == "right" or direction == "back":
                q[-1] = q[-1] - pi / 2

            if abs(np.linalg.norm(err)) <= accuracy:

                absolute = np.absolute(q[1]) + (pi - np.absolute(q[2]))
                q[-1] = -1 * (1.57 - (9.4248 - absolute - 2 * pi))

                if direction == "top":
                    q[-1] = q[-1]
                if direction == "bottom":
                    q[-1] = q[-1] + pi

                if direction == "left" or direction == "front":
                    q[-1] = q[-1] + pi / 2

                if direction == "right" or direction == "back":
                    q[-1] = q[-1] - pi / 2
                # print("Absolute: {}".format(absolute))
                # print("Absolute Degrees: {}".format(absolute * 180 / pi))
                # print("Angle Dif: {}".format(q[-1] * 180 / pi))

                return q
        raise ValueError("Could not find solution.")

    # Units: inches, radians
    # inputs should be in the global reference frame
    def ikin(self, goalPos, gamma, phi, baseID, simHuh=False, elbow_up=1, inching=True):

        if self.DEBUG:
            print(f"(Ikin)goalPos:{goalPos} Gamma:{gamma} Phi:{phi} baseID:{baseID}")
        # Robot Parameters
        # L1 = 4.125  # L1 in inches
        # L2 = 6.43  # L2 in inches
        # blockWidth = 3
        L1 = self.links[0].length
        L2 = self.links[1].length

        relativePos, localGamma = self.handlePlaneChanges(
            goalPos=goalPos, gamma=gamma, baseID=baseID, inching=inching
        )
        # x, y, z, dummy = relativePos * blockWidth  # dummy value should always 1
        x, y, z, dummy = relativePos  # dummy value should always 1

        if self.DEBUG:
            print(f"x y z: {x} {y} {z}")
            print(f"gamma: {gamma}")
            print(f"localGamma: {localGamma}")
            print(f"relativePos: {relativePos}")

        q1 = atan2(y, x)  # joint1 angle
        if q1 < -179.8:
            q1 = 0

        new_z = z - L1  # take away the height of the first link (vertical)
        new_x = x / cos(q1)

        x3 = new_x - L1 * cos(localGamma)
        z3 = new_z - L1 * sin(localGamma)  # reduce to the 2dof planar robot

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
        q5 = phi - q1
        q = np.array([q1, q2, q3, q4, q5])

        # check which ee is requested and flip angles accordingly
        if not simHuh:
            if baseID == "D":
                q = np.array([q5, q4, q3, q2, q1])

        if self.DEBUG:
            print(f"ikin output q: {q}\n\n")
        return q

    def handlePlaneChanges(self, goalPos, gamma, baseID, inching=True):

        if self.DEBUG:
            print(f"Pre plane handling:\n AEE_POSE: {self.AEE_POSE}")
            print(f"DEE POS: {self.DEE_POSE}")

        relativePos = np.zeros(3)
        baseOri = np.zeros(3)
        basePos = np.zeros(3)
        goalRot = np.zeros(3)

        # gets the relativePos, basePos, and baseOri in the global reference frame
        if baseID == "A":  # requested ee is A
            basePos = self.AEE_POSE[:3, 3].T
            baseOri = R.from_matrix(self.AEE_POSE[:3, :3]).as_euler("xyz", degrees=True)
        elif baseID == "D":  # requested ee is D
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
        if inching:
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
        else:
            if baseID == "D":  # requested ee is A, update D to match goal
                # DEEPOS = goalPos
                # DEEORI = goalOri
                self.DEE_POSE[:3, 3] = np.array(goalPos)
                self.DEE_POSE[:3, :3] = goalRot[:3, :3]
            elif baseID == "A":  # requested ee is D, update A to match goal
                # AEEPOS = goalPos
                # AEEORI = goalOri
                self.AEE_POSE[:3, 3] = np.array(goalPos)
                self.AEE_POSE[:3, :3] = goalRot[:3, :3]

        if self.DEBUG:
            print(f"Post plane handling:\n AEE_POSE: {self.AEE_POSE}")
            print(f"DEE POS: {self.DEE_POSE}")
        return relativePos, localGamma

    def getRx(self, theta):
        T = np.eye(4)
        T[1, :] = [0, cos(theta), -sin(theta), 0]
        T[2, :] = [0, sin(theta), cos(theta), 0]
        return T

    def getRy(self, theta):
        T = np.eye(4)
        T[0, :] = [cos(theta), 0, sin(theta), 0]
        T[2, :] = [-sin(theta), 0, cos(theta), 0]
        return T

    def getRz(self, theta):
        T = np.eye(4)
        T[0, :] = [cos(theta), -sin(theta), 0, 0]
        T[1, :] = [sin(theta), cos(theta), 0, 0]
        return T

    def resetEEStartingPoses(self):
        # self.AEEPOS = np.array([0,0,0]).T
        # self.AEEORI = np.array([0,0,1]).T
        # self.DEEPOS = np.array([2,0,0]).T
        # self.DEEORI = np.array([0,0,1]).T
        self.AEE_POSE = np.matmul(np.array(np.eye(4)), self.getRz(np.pi / 2))
        self.DEE_POSE = np.matmul(np.array(np.eye(4)), self.getRz(np.pi / 2))
        # x y z position
        self.AEE_POSE[:, 3] = np.array([1.5, 0.5, 5, 1])
        self.DEE_POSE[:, 3] = np.array([3.5, 0.5, 5, 1])

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

    def ikine(self, T, q0=None, unit="rad"):
        """
        Calculates inverse kinematics for homogeneous transformation matrix using numerical optimisation method.
        :param T: homogeneous transformation matrix.
        :param q0: initial list of joint angles for optimisation.
        :param unit: preferred unit for returned joint angles. Allowed values: 'rad' or 'deg'.
        :return: a list of 6 joint angles.
        """
        assert T.shape == (4, 4)
        bounds = [(link.qlim[0], link.qlim[1]) for link in self]
        reach = 0
        for link in self:
            reach += abs(link.a) + abs(link.d)
        omega = np.diag([1, 1, 1, 3 / reach])
        if q0 is None:
            q0 = np.asmatrix(np.zeros((1, self.length)))

        def objective(x):
            return (
                np.square(
                    ((np.linalg.lstsq(T, self.fkine(x))[0]) - np.asmatrix(np.eye(4, 4)))
                    * omega
                )
            ).sum()

        sol = minimize(objective, x0=q0, bounds=bounds)
        if unit == "deg":
            return np.asmatrix(sol.x * 180 / pi)
        else:
            return np.asmatrix(sol.x)

    def plot(self, stance, update, unit="rad"):
        """
        Plots the SerialLink object in a desired stance.
        :param stance: list of joint angles for SerialLink object.
        :param unit: unit of input angles.
        :return: null.
        """

        assert type(stance) is np.matrix

        if unit == "deg":
            stance = stance * np.pi / 180

        self.pipeline = VtkPipeline()
        (
            self.pipeline.reader_list,
            self.pipeline.actor_list,
            self.pipeline.mapper_list,
        ) = self.__setup_pipeline_objs()

        # print("LENGHT OF ACTOR LIST: {}".format(len(self.pipeline.actor_list)))
        self.fkine(
            stance, apply_stance=True, actor_list=self.pipeline.actor_list, update=update,
        )

        self.update_angles(stance.tolist()[0])

        # print(self.get_current_joint_config(unit='deg'))
        # print(self.end_effector_position())

        # for i in self.links:
        #     i.display(unit='deg')
        # self.pipeline.add_actor(axesCube(self.pipeline.ren))
        cube_axes = axesCubeFloor(
            self.pipeline.ren,
            self.param.get("cube_axes_x_bounds"),
            self.param.get("cube_axes_y_bounds"),
            self.param.get("cube_axes_z_bounds"),
            self.param.get("floor_position"),
        )

        self.pipeline.add_actor(cube_axes)

        for i, each in enumerate(self.pipeline.actor_list):
            each.SetScale(self.scale)
        self.__setup_structure_display()
        self.pipeline.add_actor(self._display_path())

        xyzLabels = ["X", "Y", "Z"]
        scale = [1.0, 1.0, 1.0]
        axes = MakeAxesActor(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)
        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(self.pipeline.iren)
        om2.EnabledOn()

        self.pipeline.render()

    def __setup_pipeline_objs(self):
        """
        Internal function to initialise vtk objects.
        :return: reader_list, actor_list, mapper_list
        """
        print("STL FILES: {}".format(self.stl_files))
        reader_list = [0] * len(self.stl_files)
        actor_list = [0] * len(self.stl_files)
        # print("Actor List: {}".format(actor_list))

        mapper_list = [0] * len(self.stl_files)
        for i in range(len(self.stl_files)):
            reader_list[i] = vtk.vtkSTLReader()
            loc = pkg_resources.resource_filename(
                "robopy", "/".join(("media", self.name, self.stl_files[i]))
            )
            # print(loc)
            reader_list[i].SetFileName(loc)
            mapper_list[i] = vtk.vtkPolyDataMapper()
            mapper_list[i].SetInputConnection(reader_list[i].GetOutputPort())
            actor_list[i] = vtk.vtkActor()
            actor_list[i].SetMapper(mapper_list[i])
            actor_list[i].GetProperty().SetColor(self.colors[i])  # (R,G,B)

        return reader_list, actor_list, mapper_list

    def _add_block(self, position):
        reader_list = vtk.vtkSTLReader()
        loc = pkg_resources.resource_filename(
            "robopy", "/".join(("media", self.name, "block.stl"))
        )
        # print(loc)
        reader_list.SetFileName(loc)
        mapper_list = vtk.vtkPolyDataMapper()
        mapper_list.SetInputConnection(reader_list.GetOutputPort())
        actor_list = vtk.vtkActor()
        actor_list.SetMapper(mapper_list)
        color = vtk_named_colors(["Purple"])

        actor_list.GetProperty().SetColor(color[0])  # (R,G,B)
        actor_list.SetScale(0.013)
        actor_list.SetPosition(position)
        # print("Adding block at pos: {}".format(position))
        self.pipeline.add_actor(actor_list)

        return actor_list, reader_list, mapper_list

    def _display_path(
        self,
        path=[
            (0, 0, 0, "top"),
            (1, 0, 0, "top"),
            (2, 0, 0, "top"),
            (3, 0, 0, "top"),
            (4, 0, 0, "top"),
            (5, 0, 0, "top"),
        ],
    ):
        for point in path:
            prop_assembly = cubeForPath(point)
            self.pipeline.add_actor(prop_assembly)

    @staticmethod
    def _setup_file_names(num):
        file_names = []
        for i in range(0, num):
            file_names.append("link" + str(i) + ".stl")

        return file_names

    def animate(
        self,
        stances,
        unit="rad",
        frame_rate=25,
        gif=None,
        num_steps=None,
        orientation=None,
        showPath=True,
        showPlacedBlock=True,
        showTrajectory=True,
        update=None,
    ):
        """
        Animates SerialLink object over nx6 dimensional input matrix, with each row representing list of 6 joint angles.
        :param stances: nx6 dimensional input matrix.
        :param unit: unit of input angles. Allowed values: 'rad' or 'deg'
        :param frame_rate: frame_rate for animation. Could be any integer more than 1. Higher value runs through
        stances faster.
        :return: null
        """

        if unit == "deg":
            stances = stances * np.pi / 180

        # print(stances.shape[0]-1)
        # print(stances.shape)
        self.pipeline = VtkPipeline(total_time_steps=stances.shape[0] - 1, gif_file=gif)
        (
            self.pipeline.reader_list,
            self.pipeline.actor_list,
            self.pipeline.mapper_list,
        ) = self.__setup_pipeline_objs()
        self.fkine(
            stances,
            apply_stance=True,
            actor_list=self.pipeline.actor_list,
            directions=None,
            update=update[0],
        )
        self.update_angles(stances.tolist()[0])

        # print(self.get_current_joint_config(unit='deg'))
        # print(self.end_effector_position())
        # self.pipeline.add_actor(axesCube(self.pipeline.ren))

        cube_axes = axesCubeFloor(
            self.pipeline.ren,
            self.param.get("cube_axes_x_bounds"),
            self.param.get("cube_axes_y_bounds"),
            self.param.get("cube_axes_z_bounds"),
            self.param.get("floor_position"),
        )

        self.pipeline.add_actor(cube_axes)
        previous_point = []
        previous_path = []

        def execute(obj, event):
            nonlocal stances, previous_point
            self.pipeline.timer_tick()
            timer = self.pipeline.timer_count
            index = timer / num_steps

            self.fkine(
                stances,
                apply_stance=True,
                actor_list=self.pipeline.actor_list,
                timer=self.pipeline.timer_count,
                num_steps=num_steps,
                orientation=orientation,
                update=update[int(index)],
            )

            self.update_angles(stances[self.pipeline.timer_count].tolist()[0])
            # timer = self.pipeline.timer_count
            # index = (timer / num_steps)

            # if index == 3:
            #     for i in range(4):
            #         self.pipeline.actor_list[i].GetProperty().SetColor(0, 0, 1)
            if update is not None:
                for animation_update in update:
                    if animation_update.index == index:
                        if animation_update.placedObstacle and showPlacedBlock:
                            self._add_block(animation_update.obstacle)
                        if showPath:
                            for i in previous_path:
                                self.pipeline.remove_actor(i)
                                previous_path.remove(i)

                            for point in animation_update.path:
                                previous_path.append(
                                    self.pipeline.add_actor(cubeForPath(point))
                                )

                        if showTrajectory:
                            for i in previous_point:
                                self.pipeline.remove_actor(i)
                                previous_point.remove(i)

                            for index, point in enumerate(animation_update.trajectory):
                                previous_point.append(
                                    self.pipeline.add_actor(
                                        circleForTrajectory(
                                            point,
                                            animation_update.direction,
                                            index=index,
                                        )
                                    )
                                )
                        self.pipeline.animate()

            self.pipeline.iren = obj
            self.pipeline.iren.GetRenderWindow().Render()

        self.pipeline.iren.AddObserver("TimerEvent", execute)
        self.__setup_structure_display()
        # if display_path:
        #     self.pipeline.add_actor(self._display_path())

        xyzLabels = ["X", "Y", "Z"]
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

    # def get_end_effector_pos(self, stance):
    #     return self.fkine(stance)

    # def flip_base(self, ee_pos, direction, value):
    #     # ee_pos[0] = math.floor(ee_pos[0])
    #     # ee_pos[2] = round(ee_pos[2])
    #
    #     if direction == "top" or direction == "bottom":
    #         # ee_pos[0] = ee_pos[0] + 0.5
    #         new_base = tr.trotz(value, unit="deg", xyz=ee_pos)
    #
    #     if direction == "left" or direction == "right":
    #         # ee_pos[0] = ee_pos[0] + 0.5
    #         # ee_pos[2] = ee_pos[2]
    #         new_base = tr.troty(value, unit="deg", xyz=ee_pos)
    #
    #     if direction == "front" or direction == "back":
    #         new_base = tr.trotx(value, unit="deg", xyz=ee_pos)
    #
    #     return new_base


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
