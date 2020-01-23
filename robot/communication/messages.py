from robot.common.states import RobotBehaviors


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


class MessageWrapper:
    def __init__(self, topic, message):
        self.topic = topic
        self.message = message
