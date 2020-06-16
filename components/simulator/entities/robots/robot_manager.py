from components.simulator.entities.robots.robot_entity import Robot
from components.simulator.globals import robot_queue


class RobotManager:
    """
    Manager class for all robot entities. It calls each robot to update, as well as deals with the creation and
    removal of all robot entities. 

    """

    def __init__(self, new_actors):
        self.new_actors = new_actors
        self.robot_actors = {}

    def update(self, pipeline):
        while not self.new_actors.empty():
            topic, message, queue = self.new_actors.get()
            self.add_new_robot(queue, topic, pipeline)

        for robot_id in self.robot_actors:
            self.robot_actors[robot_id].update(pipeline)

    def add_new_robot(self, result_queue, robot_id, pipeline):
        self.robot_actors[robot_id] = Robot(pipeline, result_queue, robot_id)

    def get_robot_count(self):
        return len(self.robot_actors.keys())


# NOTE: The manager is defined here
robotManager = RobotManager(robot_queue)
