import numpy as np


def determine_ferry_regions(level, num_rows, num_cols, direction=("FRONT","RIGHT"),
                            ferry_region_size=3, block_placed=1, ):
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

    starting_row_index, starting_column_index = 0, 0


    row = num_rows
    column = num_cols

    level_ferry_regions = [['0'] * num_cols for i in range(num_rows)]
    x = 'X'

    count = 1
    while starting_row_index < num_rows and starting_column_index < num_cols:

        if "FRONT" in direction:
            for i in range(starting_column_index, num_cols):
                if level[starting_row_index][i] == block_placed:
                    level_ferry_regions[starting_row_index][i] = x
            starting_row_index += 1

        if "RIGHT" in direction:
            for i in range(starting_row_index, num_rows):
                if level[i][num_cols - 1] == block_placed:
                    level_ferry_regions[i][num_cols - 1] = x
            num_cols -= 1

        if "BACK" in direction:
            if starting_row_index < num_rows:
                for i in range(num_cols - 1, starting_column_index - 1, -1):
                    if level[num_rows - 1][i] == block_placed:
                        level_ferry_regions[num_rows - 1][i] = x
                num_rows -= 1

        if "LEFT" in direction:
            if starting_column_index < num_cols:
                for i in range(num_rows - 1, starting_row_index - 1, -1):
                    if level[i][starting_column_index] == block_placed:
                        level_ferry_regions[i][starting_column_index] = x
                starting_column_index += 1

        count += 1

        x = 'X' if (x == 'X' and count <= ferry_region_size) else '0'

    # Print the filled matrix
    print(f"Output for rows = {row}, columns = {column}")
    for i in range(row):
        for j in range(column):
            print(level_ferry_regions[i][j], end=" ")
        print()

    return np.array(level_ferry_regions)


if __name__ == '__main__':

    level = np.array([[1, 1, 1, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1],
             [1, 1, 1, 1, 1, 1, 1]])

    # level = np.array([[0, 0, 0],
    #    [0, 0, 0],
    #    [0, 0, 0],
    #    [0, 0, 1],
    #    [0, 1, 1],
    #    [0, 1, 1]])

    m, n = level.shape
    determine_ferry_regions(level, m, n)
