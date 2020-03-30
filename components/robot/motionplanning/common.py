import math

import numpy as np
import numpy.testing as npt

import components.robot.motionplanning.transforms as tr
from components.robot.motionplanning import check_args


def ishomog(tr, dim, rtest=""):
    """ISHOMOG Test if SE(3) homogeneous transformation matrix.
    ISHOMOG(T) is true if the argument T is of dimension 4x4 or 4x4xN, else false.
    ISHOMOG(T, 'valid') as above, but also checks the validity of the rotation sub-matrix.
    See Also: isrot, ishomog2, isvec"""
    try:
        assert type(tr) is np.matrix, "Argument should be a numpy matrix"
        assert dim == (3, 3) or dim == (4, 4)
    except AssertionError:
        return False
    is_valid = None
    if rtest == "valid":
        is_valid = lambda matrix: abs(np.linalg.det(matrix) - 1) < np.spacing([1])[0]
    flag = True
    if check_args.is_mat_list(tr):
        for matrix in tr:
            if not (matrix.shape[0] == dim[0] and matrix.shape[1] == dim[0]):
                flag = False
        # if rtest = 'valid'
        if flag and rtest == "valid":
            flag = is_valid(
                tr[0]
            )  # As in matlab code only first matrix is passed for validity original
            # TODO-Do we need to original all matrices in list for validity of rotation submatrix -- Yes
    elif isinstance(tr, np.matrix):
        if tr.shape[0] == dim[0] and tr.shape[1] == dim[0]:
            if flag and rtest == "valid":
                flag = is_valid(tr)
        else:
            flag = False
    else:
        raise ValueError("Invalid data type passed to common.ishomog()")
    return flag


def isvec(v, length=3):
    """
    ISVEC Test if vector
    """
    assert type(v) is np.matrix
    d = v.shape
    h = len(d) == 2 and min(v.shape) == 1 and v.size == length

    return h


def isrot(rot, dtest=False):
    """
    ISROT Test if SO(2) or SO(3) rotation matrix
    ISROT(rot) is true if the argument if of dimension 2x2, 2x2xN, 3x3, or 3x3xN, else false (0).
    ISROT(rot, 'valid') as above, but also checks the validity of the rotation.
    See also  ISHOMOG, ISROT2, ISVEC.
    """
    if type(rot) is np.matrix:
        rot = [rot]
    if type(rot) is list:
        for each in rot:
            try:
                assert type(each) is np.matrix
                assert each.shape == (3, 3)
                npt.assert_almost_equal(np.linalg.det(each), 1)
            except AssertionError:
                return False
    return True


def isrot2(rot, dtest=False):
    if type(rot) is np.matrix:
        rot = [rot]
    if type(rot) is list:
        for each in rot:
            try:
                assert type(each) is np.matrix
                assert each.shape == (2, 2)
                npt.assert_almost_equal(np.linalg.det(each), 1)
            except AssertionError:
                return False
    return True


def get_rotation_from_homogeneous_transform(transform):
    """Extract the rotation section of the homogeneous transformation

    :param transform: The 4x4 homogeneous transform to extract the
                      rotation matrix from.
    :type transform: numpy.ndarray

    :returns: 3x3 Rotation Matrix
    :rtype: numpy.matrix
    """
    s = transform.shape
    if s[0] != s[1]:
        raise ValueError("Matrix must be a 4x4 homogenous transformation", s)
    n = s[0]
    rotation = transform[0: n - 1, 0: n - 1]
    return rotation


def create_homogeneous_transform_from_point(p):
    """Create a homogeneous transform to move to a given point

    :param p: The (x, y, z) point we want our homogeneous tranform to move to
    :type p: numpy.ndarray

    :returns: 4x4 Homogeneous Transform of a point
    :rtype: numpy.matrix
    """
    Identity = np.identity(3)
    Transform = np.vstack((Identity, np.zeros((3,))))
    p = np.hstack((p, np.ones(1))).reshape((4, 1))
    Transform = np.hstack((Transform, p))
    return Transform


def create_point_from_homogeneous_transform(T):
    """Create a point from a homogeneous transform

    :param T: The 4x4 homogeneous transform
    :type T: numpy matrix

    :returns: The (x, y, z) coordinates of a point from a transform
    :rtype: np.ndarray
    """
    return T[0:3, 3]


# def round_end_effector_position(ee_pos, direction, offset=None):
#
#
#     if direction == "top" or direction == "bottom":
#         ee_pos[0] = math.floor(ee_pos[0]) + 0.49
#         ee_pos[2] = round(ee_pos[2])
#
#
#     if direction == "left" or direction == "right":
#         ee_pos[0] = math.floor(ee_pos[0]) + 0.49
#
#         ee_pos[2] = round(ee_pos[2])
#
#     if direction == "front" or direction == "back":
#         ee_pos[0] = math.floor(ee_pos[0])
#         ee_pos[2] = round(ee_pos[2])
#         ee_pos[0] = ee_pos[0] + 0.49
#
#     return ee_pos


def round_end_effector_position(ee_pos, direction, point, offset=None):
    if point is None:
        print("POINT IS NONE")
        print(ee_pos)
        if direction == "top" or direction == "bottom":
            ee_pos[0] = math.floor(ee_pos[0]) + 0.5
            ee_pos[2] = round(ee_pos[2])

        if direction == "left" or direction == "right":
            ee_pos[0] = math.ceil(ee_pos[0]) - 0.5

            ee_pos[2] = round(ee_pos[2])

        if direction == "front" or direction == "back":
            ee_pos[0] = math.floor(ee_pos[0])
            ee_pos[2] = round(ee_pos[2])
            ee_pos[0] = ee_pos[0] + 0.5
        return ee_pos

    ee_pos = np.copy(point)
    return ee_pos


def flip_base(ee_pos, direction, value, animation=False):
    # ee_pos = np.copy(ee_pos).tolist()[0]
    # ee_pos[0] = math.floor(ee_pos[0])
    # ee_pos[2] = round(ee_pos[2])

    if direction == "top" or direction == "bottom":
        # ee_pos[0] = ee_pos[0] + 0.5
        new_base = tr.trotz(value, unit="deg", xyz=ee_pos)
        # if animation:
        #     new_base[0, 3] = new_base[0, 3] + 0.5 #+0.5
        #     new_base[2, 3] = new_base[2, 3] + 0.5 #+0.5

    if direction == "left" or direction == "right":
        # ee_pos[0] = ee_pos[0] + 0.5
        # ee_pos[2] = ee_pos[2]
        new_base = tr.troty(value, unit="deg", xyz=ee_pos)

        new_base = tr.troty(-90, unit="deg", xyz=ee_pos)

        new_pos = create_point_from_homogeneous_transform(new_base)
        new_base = new_base * tr.trotz(value, unit="deg", xyz=ee_pos)

        new_base[0:3, 3] = new_pos
        # if animation:
        #     new_base[0, 3] = new_base[0, 3] + 0.5 #+0.5
        #     new_base[2, 3] = new_base[2, 3] + 0.5 #+0.5
        # else:
        new_base[0, 3] = new_base[0, 3] + 1.37  # +1.37
        new_base[2, 3] = new_base[2, 3] + 1.37  # +1.37

    if direction == "front" or direction == "back":
        new_base = tr.trotx(value, unit="deg", xyz=ee_pos)

    return new_base
