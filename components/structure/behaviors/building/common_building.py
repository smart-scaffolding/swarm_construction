from functools import total_ordering
from uuid import uuid4

import numpy as np


@total_ordering
class Division:
    def __init__(
        self,
        x_range,
        y_range,
        z_range,
        id=None,
        wavefront_order=None,
        status="UNCLAIMED",
        owner=None,
        centroid=None,
        child=None,
        direction=None,
        pos=None,
        num_blocks=9,
        path_to_node=None,
    ):
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.status = status
        self.owner = owner
        self.centroid = self.calculate_centroid() if centroid is None else centroid
        self.area = self.area()
        self.order = wavefront_order
        if id:
            self.id = id
        else:
            self.id = ("DIVISION_" + str(uuid4())).encode()
        self.children = {child}  # used for notifying other divisions when done
        if direction is None:
            self.direction = set()
        else:
            self.direction = {direction}  # expected to be only right and up
        self.num_blocks = num_blocks  # number of blocks to place in goal node
        self.pos = self.centroid
        self.path_to_node = path_to_node

    def __eq__(self, other):
        if self.id == other.id:
            if None in self.children or None in other.children:
                return False
        else:
            return self.id == other.id

    def __lt__(self, other):
        if self.id == other.id:
            if None in self.children:
                return True
            else:
                return False
        else:
            return self.order < other.order

    def __repr__(self):
        return (
            f"\nID: {self.id}\nOrder: {self.order}\n\tChildren: {self.children}\n\tDirection: {self.direction}\n\t"
            f"Num Blocks: {self.num_blocks}\n\tPosition: {self.pos}\n\tX_Range: {self.x_range}"
            f"\n\tY_Range: {self.y_range}\n\tZ_Range: {self.z_range}\n\t\tCentroid: {self.centroid}\n\t\tArea: "
            f"{self.area}\n\n\n\tPath to node: {self.path_to_node}"
        )

    def change_status(self, new_status):
        self.status = new_status

    def claim(self, owner):
        self.owner = owner

    def calculate_centroid(self):
        x = (self.x_range[0] + self.x_range[1]) / 2
        y = (self.y_range[0] + self.y_range[1]) / 2
        z = (self.z_range[0] + self.z_range[1]) / 2
        return (x, y, z)

    def area(self):
        x_start, x_end = self.x_range
        y_start, y_end = self.y_range
        z_start, z_end = self.z_range

        x = x_end - x_start
        y = y_end - y_start
        z = z_end - z_start

        return x * y * z

    def __add__(self, other):
        x = self.x_range + other.x_range
        y = self.y_range + other.y_range
        z = self.z_range + other.z_range

        self.x_range = x
        self.y_range = y
        self.z_range = z
        return self

    def __hash__(self):
        return hash(self.id)


class Block:
    def __init__(self, final_destination, assigned=1, id=None):
        self.location = None
        self.next_destination = None
        self.final_destination = final_destination
        if id:
            self.id = id
        else:
            self.id = ("BLOCK_" + str(uuid4())).encode()
        self.assigned_node = assigned

    def set_next_location(self, new_location):
        if self.location is None:
            self.location = (0, 0, 1)
        else:
            self.location = self.next_destination
        self.next_destination = new_location
        return self

    def __repr__(self):
        return (
            f"\n\nBlock: {self.id}\n\tCurrent Loc: {self.location}\n\tNext Loc: {self.next_destination}\n\tFinal "
            f"Dest: {self.final_destination}"
        )


@total_ordering
class Robot:
    def __init__(self, id, pos, claimed_division=None):
        self.id = id
        self.pos = pos
        self.closest_points = []
        self.target = None
        self.desired_target = None
        self.desired_target_distance = None
        self.claimed_division = claimed_division
        self.status = None

    def update_status(self, status):
        self.status = status

    def find_new_target(self, points):
        # print(f"Unfiltered closest points: {self.closest_points}")
        self.closest_points.remove((self.desired_target_distance, self.desired_target))
        # print(f"Filtered closest points: {self.closest_points}")

        arg_min_score = np.argmin(np.array(self.closest_points)[:, 0])
        self.desired_target = self.closest_points[arg_min_score][1]
        self.desired_target_distance = self.closest_points[arg_min_score][0]
        # print(f"New target: {self.desired_target.pos}")

    def _is_valid_operand(self, other):
        return hasattr(other, "desired_target_distance")

    def __eq__(self, other):
        if not self._is_valid_operand(other):
            return NotImplemented
        return self.desired_target_distance == other.desired_target_distance

    def __lt__(self, other):
        if not self._is_valid_operand(other):
            return NotImplemented
        # if self.desired_target_distance == other.desired_target_distance:
        #     count = 1
        #     while self.closest_points[count][1].pos == other.closest_points[count][1].pos:
        #         count+=1
        #     return self.closest_points[count][1].pos < other.closest_points[count][1].pos

        return self.desired_target_distance < other.desired_target_distance

    def __repr__(self):
        return (
            f"\n Robot ID: {self.id}\n\tPos: {self.pos}\n\tClaimed: {self.claimed_division}"
            f"\n\tTarget: {self.target}\n\tDesired Target: {self.desired_target}"
            f"\n\tClosest points: {self.closest_points}"
        )

    def __hash__(self):
        return hash(self.id)


def spiral_sort():
    sorted_blueprint = []

    # blueprint = np.array([[[1, 2, 3], [4, 5, 6]],
    #      [[7, 8, 9], [10, 11, 12]],
    #      [[13, 14, 15], [16, 17, 18]]])

    blueprint = np.array([[1, 2, 3, 4, 5], [6, 7, 8, 9, 10], [11, 12, 13, 14, 15]])
    # print(f"Blueprint: {blueprint}")
    # print(f"Blueprint size: {len(blueprint)} -- {len(blueprint[0])} -- {len(blueprint[0][0])}")
    rows = len(blueprint)
    columns = len(blueprint[0])
    # for index in range(len(blueprint[0][0])):
    layer = spiral_sort_helper(rows, columns, blueprint[:, :])
    layer = layer[::-1]
    sorted_blueprint.append(layer)
    # for block in layer:
    #     print(f"Spiral sorted block at {block.position[0]}-{block.position[1]}-{block.position[2]}")
    print(sorted_blueprint)
    return sorted_blueprint[::-1]


def spiral_sort_helper(
    m, n, a, level=1, x_offset=0, y_offset=0, z_offset=0, block_placed=1
):
    sorted_array = []
    new_positions = []
    k = 0
    starting_column_index = 0

    """
    k - starting row index
    m - ending row index
    l - starting column index
    n - ending column index
    i - iterator
    """

    while k < m and starting_column_index < n:

        for i in range(starting_column_index, n):
            # print(a[k][i], end=" ")
            if a[k][i] == block_placed:
                sorted_array.append(a[k][i])
                print(f"{a[k][i]}: ({k},{i})")
                new_positions.append((k + x_offset, i + y_offset, level + z_offset))
        k += 1

        for i in range(k, m):
            # print(a[i][n - 1], end=" ")
            if a[i][n - 1] == block_placed:
                sorted_array.append(a[i][n - 1])
                print(f"{a[i][n-1]}: ({i},{n-1})")
                new_positions.append((i + x_offset, n - 1 + y_offset, level + z_offset))

        n -= 1

        if k < m:

            for i in range(n - 1, (starting_column_index - 1), -1):
                # print(a[m - 1][i], end=" ")
                if a[m - 1][i] == block_placed:
                    sorted_array.append(a[m - 1][i])
                    print(f"{a[m-1][i]}: ({m-1},{i})")
                    new_positions.append(
                        (m - 1 + x_offset, i + y_offset, level + z_offset)
                    )
            m -= 1

        if starting_column_index < n:
            for i in range(m - 1, k - 1, -1):
                # print(a[i][l], end=" ")
                if a[i][starting_column_index] == block_placed:
                    sorted_array.append(a[i][starting_column_index])
                    print(f"{a[i][starting_column_index]}: ({i},{starting_column_index})")
                    new_positions.append((i + x_offset, starting_column_index + y_offset, level + z_offset))
            starting_column_index += 1
    return sorted_array, new_positions


if __name__ == "__main__":
    blueprint2 = np.array([[[1] * 1] * 10] * 10)

    blueprint2[0, :, :] = 0
    blueprint2[1, :, :] = 0
    blueprint2[-1, :, :] = 0
    blueprint2[-2, :, :] = 0
    blueprint2[:, 0, :] = 0
    blueprint2[:, 1, :] = 0
    blueprint2[:, -1, :] = 0
    blueprint2[:, -2, :] = 0

    m, n, _ = blueprint2.shape
    array, positions = spiral_sort_helper(m, n, blueprint2[:, :, 0])
    print(positions)
