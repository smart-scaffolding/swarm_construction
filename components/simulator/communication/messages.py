from components.robot.common.states import RobotBehaviors


class Message:
    def __init__(self, message_id):
        self.message_id = message_id

    def __str__(self):
        return str(self.__dict__)


class FerryBlocksMessage(Message):
    def __init__(self, blocks_to_move):
        super().__init__(message_id=RobotBehaviors.FERRY)
        self.blocks_to_move = blocks_to_move


class BuildMessage(Message):
    def __init__(self, blocks_to_move):
        super().__init__(message_id=RobotBehaviors.BUILD)
        self.blocks_to_move = blocks_to_move


class WaitMessage(Message):
    def __init__(self):
        super().__init__(message_id=RobotBehaviors.WAIT)


class MoveToPointMessage(Message):
    def __init__(self, destination):
        super().__init__(message_id=RobotBehaviors.MOVE)
        self.location_to_move_to = destination


class StatusUpdateMessage(Message):
    def __init__(self, status, payload):
        super(StatusUpdateMessage, self).__init__(message_id=RobotBehaviors.UPDATE)
        self.robot_status = status
        self.payload = payload


class StatusUpdateMessagePayload:
    def __init__(self, robot_base):
        self.robot_base = robot_base


class PlacedBlockUpdateMessagePayload:
    def __init__(self, robot_base, block_placed):
        self.robot_base = robot_base
        self.block_placed = block_placed


class BlockLocationMessage:
    def __init__(self, block_id, location):
        self.id = block_id
        self.location = location


class AnimationUpdateMessage(Message):
    def __init__(
        self,
        robot_base,
        direction=None,
        trajectory=None,
        path=None,
        placedObstacle=False,
        obstacle=None,
        block_on_ee=False,
        debug_text="",
    ):
        super().__init__(message_id=RobotBehaviors.SIMULATION)
        self.robot_base = robot_base
        self.direction = direction
        self.trajectory = trajectory
        self.path = path
        self.placedObstacle = placedObstacle
        self.obstacle = obstacle
        self.block_on_ee = block_on_ee
        self.debug_text = debug_text


class FerryBlocksStatusFinished(Message):
    def __init__(self):
        super().__init__(message_id=RobotBehaviors.FERRY)
        self.finished = True


class MessageWrapper:
    def __init__(self, topic, message):
        self.topic = topic
        self.message = message
