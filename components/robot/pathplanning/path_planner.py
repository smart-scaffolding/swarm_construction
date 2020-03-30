from components.robot.common.states import PathPlanners
from components.robot.pathplanning.searches.a_star import AStar
from components.robot.pathplanning.searches.face_star import FaceStar


class PathPlanner:
    def __init__(self, blueprint, arm_reach, search=PathPlanners.FaceStar):
        self.blueprint = blueprint
        self.arm_reach = arm_reach
        if search == PathPlanners.FaceStar:
            self.planner = FaceStar(self.blueprint, self.arm_reach)
        if search == PathPlanners.AStar:
            self.planner = AStar(self.blueprint)

    def get_path(self, start, goal):
        return self.planner.get_path(start=tuple(start), goal=tuple(goal))


# class PathPlannerImp:
#     def get_path(self, start, goal):
#         raise NotImplemented("Must overwrite get_path to return a path")
