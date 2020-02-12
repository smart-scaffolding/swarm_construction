import numpy as np
from copy import copy
from components.structure.behaviors.building.common_building import Block


def determine_ferry_regions(level, num_rows, num_cols, direction=("RIGHT", "FRONT"),
                            ferry_region_size=5, block_placed=1, x_offset=0, y_offset=0, z_offset=1, num_blocks={
            "FRONT": 25,
                                                                                                     "RIGHT": 25,
            "BACK": 0, "LEFT": 0}, print_output=False):
    """
    Used to determine the locations in a 2d array where blocks can be placed to be ferried

    :param level: 2d Numpy array denoting a level within the 3d structure
    :param num_rows: The height of the level
    :param num_cols: The width of the level
    :param direction: The direction to place blocks (Front, Back, Left, Right)
    :param ferry_region_size: Size of the area to mark for placing blocks to be ferried
    :param block_placed: Marker to look for in level that denotes a block exists at that location
    :return: 2d Numpy array same size as level with 'X' marking region to blocks can be placed (ferried)
    """

    print("\n\n\nIn ferry regions")
    print(f"Num Blocks: {num_blocks}")
    print(f"Offsets: {x_offset} {y_offset} {z_offset}")
    print(f"Direction: {direction}")
    starting_row_index, starting_column_index = 0, 0


    row = copy(num_rows)
    column = copy(num_cols)

    level_ferry_regions = [[0] * num_cols for i in range(num_rows)]
    x = 'X'

    # count = 1

    num_blocks_to_place = np.sum(np.asarray(list(num_blocks.values())))


    new_block_locations = {
        "FRONT": [],
        "RIGHT": [],
        "BACK": [],
        "LEFT": []
    }

    while num_blocks_to_place > 0:
        starting_row_index, starting_column_index = 0, 0
        num_rows = copy(row)
        num_cols = copy(column)

        # row = copy(num_rows)
        # column = copy(num_cols)
        count = 1
        while starting_row_index < num_rows and starting_column_index < num_cols:

            if "LEFT" in direction:
                num_blocks_to_place_left = num_blocks["LEFT"]
                for i in range(starting_column_index, num_cols):
                    if num_blocks_to_place_left <= 0:
                        # print("No more blocks left to place front:")
                        break
                    if level[starting_row_index][i] == block_placed:
                        level_ferry_regions[starting_row_index][i] += 1
                        # print(f"Front: ({x_offset+ starting_row_index}, {y_offset + i}, "
                        #       f"{z_offset + level_ferry_regions[starting_row_index][i]})")

                        new_block_locations["LEFT"].append((x_offset + starting_row_index, y_offset + i, z_offset +
                                                             level_ferry_regions[starting_row_index][i]))
                        num_blocks_to_place -= 1
                        num_blocks_to_place_left -= 1
                starting_row_index += 1

            if "RIGHT" in direction:
                num_blocks_to_place_right = num_blocks["RIGHT"]
                for i in range(starting_row_index, num_rows):
                    if num_blocks_to_place_right <= 0:
                        # print("No more blocks left to place right:")
                        break
                    if level[i][num_cols - 1] == block_placed:
                        level_ferry_regions[i][num_cols - 1] += 1
                        # print(f"Right: ({x_offset + i}, {y_offset + num_cols - 1}, "
                        #       f"{z_offset + level_ferry_regions[i][num_cols - 1]})")

                        new_block_locations["RIGHT"].append((x_offset + i, y_offset + num_cols - 1,
                                                             z_offset + level_ferry_regions[i][num_cols - 1]))
                        num_blocks_to_place -= 1
                        num_blocks_to_place_right -= 1
                num_cols -= 1

            if "FRONT" in direction:
                num_blocks_to_place_front = num_blocks["FRONT"]
                if starting_row_index < num_rows:
                    for i in range(num_cols - 1, starting_column_index - 1, -1):
                        if num_blocks_to_place_front <= 0:
                            break
                        if level[num_rows - 1][i] == block_placed:
                            level_ferry_regions[num_rows - 1][i] += 1
                            new_block_locations["FRONT"].append((x_offset + num_rows - 1, y_offset +
                                                                i,
                                                                z_offset + level_ferry_regions[num_rows - 1][i]))
                            num_blocks_to_place -= 1
                            num_blocks_to_place_front -= 1
                    num_rows -= 1

            if "BACK" in direction:
                num_blocks_to_place_back = num_blocks["BACK"]
                if starting_column_index < num_cols:
                    for i in range(num_rows - 1, starting_row_index - 1, -1):
                        if num_blocks_to_place_back <= 0:
                            break
                        if level[i][starting_column_index] == block_placed:
                            level_ferry_regions[i][starting_column_index] += 1
                            new_block_locations["BACK"].append((x_offset + i, y_offset + starting_column_index,
                                                                 z_offset + level_ferry_regions[i][starting_column_index]))
                            num_blocks_to_place -= 1
                            num_blocks_to_place_back -= 1
                    starting_column_index += 1

            count += 1

            if count > ferry_region_size:
                break

            if num_blocks_to_place <= 0:
                break
            # x = 'X' if (x == 'X' and count <= ferry_region_size) else '0'

        # print("moving to next line")
        # print(f"Total: {num_blocks_to_place} front: {num_blocks_to_place_front} right: {num_blocks_to_place_right}")
        if num_blocks_to_place <= 0:
            break
    # Print the filled matrix
    if print_output:
        print(f"Output for rows = {row}, columns = {column}")
        for i in range(row):
            for j in range(column):
                print(level_ferry_regions[i][j], end=" ")
            print()

    # flattened_locations = []
    # for location in list(new_block_locations.values()):
    #
    #     flattened_locations.append(location)

    flattened_locations = [val for sublist in list(new_block_locations.values()) for val in sublist]
    flattened_locations.reverse()

    return np.array(level_ferry_regions), new_block_locations, flattened_locations


if __name__ == '__main__':

    # level = np.array([[1, 1, 1, 1, 1, 1, 0],
    #          [1, 1, 1, 1, 1, 1, 1],
    #          [1, 1, 1, 1, 1, 1, 1],
    #          [1, 0, 0, 1, 1, 1, 1],
    #          [1, 1, 1, 1, 1, 1, 1],
    #          [1, 1, 1, 1, 1, 1, 1],
    #          [1, 1, 1, 1, 1, 1, 1],
    #          [1, 1, 1, 1, 1, 1, 1],
    #          [1, 1, 1, 1, 1, 1, 1]])

    level = np.array([[1] * 5,
                     ] * 5)

    # level = np.array([[0, 0, 0],
    #    [0, 0, 0],
    #    [0, 0, 0],
    #    [0, 0, 1],
    #    [0, 1, 1],
    #    [0, 1, 1]])

    m, n = level.shape
    print(m, n)
    _, new_block_locations, flattend_locations = determine_ferry_regions(level, num_rows=m, num_cols=n, direction=(

                                                                                                       "FRONT"),
                            ferry_region_size=1, x_offset=0, y_offset=0, z_offset=1,
                            num_blocks={
                                "FRONT": 25,
                                "RIGHT": 0,
                                "BACK": 0,
                                "LEFT": 0}, print_output=True)
    print(new_block_locations)
    print(flattend_locations)
