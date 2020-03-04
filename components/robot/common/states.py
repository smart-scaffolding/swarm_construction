from enum import Enum, IntEnum


class Block(object):
    def __init__(self, position, status, final_destination):
        self.current_position = position
        self.status = status
        self.final_destination = final_destination

    def __str__(self):
        return str(self.__dict__)


class BlockFace:

    def __init__(self, xPos, yPos, zPos, face, ee_on_face, blockWidth=0.49):
        self.xPos = xPos
        self.yPos = yPos
        self.zPos = zPos
        self.face = face
        self.ee_on_face = ee_on_face
        self.blockWidth = blockWidth

    def return_tuple(self):
        return (self.xPos, self.yPos, self.zPos)

    def return_int_tuple(self):
        return (int(self.xPos), int(self.yPos), int(self.zPos))

    def get_face_coordinate(self):
        coordinate = [self.xPos, self.yPos, self.zPos]
        if self.face == 'front':
            coordinate[1] = coordinate[1] - self.blockWidth
        elif self.face == 'back':
            coordinate[1] = coordinate[1] + self.blockWidth
        elif self.face == 'left':
            coordinate[0] = coordinate[0] - self.blockWidth
        elif self.face == 'right':
            coordinate[0] = coordinate[0] + self.blockWidth
        elif self.face == 'top':
            coordinate[2] = coordinate[2] + self.blockWidth
        elif self.face == 'bottom':
            coordinate[2] = coordinate[2] - self.blockWidth
        else:
            return None
        return tuple(coordinate)

    def round_pos(self):
        self.xPos = round(self.xPos, 1)
        self.yPos = round(self.yPos, 1)
        self.zPos = round(self.zPos, 1)


class Division(object):
    def __init__(self):
        pass

    def get_locations_to_place_blocks(self):
        return [(4, 4, 4, "Top"), (5, 5, 5, "Top"), (6, 6, 6, "Top")]

    def __str__(self):
        return str(self.__dict__)


class MoveBlocksStore(object):
    def __init__(self, blocks_to_remove, division):
        self.blocks_to_remove = blocks_to_remove
        self.division = division
        self.locations_to_place_blocks = division.get_locations_to_place_blocks()

    # def __str__(self):
    #     return str(self.__dict__)


class RobotCommunicator:
    def __init__(self, robot_communicator, robot_id):
        self.robot_communicator = robot_communicator
        self.robot_id = robot_id


class PathPlanners(Enum):
    FaceStar = 0
    AStar = 1

class RobotBehaviors(Enum):
    FERRY = 0
    WAIT = 1
    MOVE = 2
    BUILD = 3
    UPDATE = 4
    SIMULATION = 5

class BuildingStates(IntEnum):
    WAITING_FOR_FERRYING = 0
    WAITING_FOR_FILLING = 1
    DONE_ORIGIN = 2
    DONE = 3
    EMPTY = 4


class BlockMoved:
    def __init__(self, location, placed_block=False):
        self.location = location
        self.placed_block = placed_block