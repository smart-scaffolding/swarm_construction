import vtk
import numpy as np
from components.simulator.model.graphics import vtk_named_colors
from components.simulator.model.model import Inchworm
from components.simulator.entities.robot_text import RobotText
from components.simulator.globals import *
from logzero import logger
from random import uniform
import pkg_resources


class Robot:
    def __init__(self, pipeline, robot_queue, robot_id, show_as_point=False):
        logger.info(f"Robot class is created: {robot_id}")
        self.robot_queue = robot_queue
        self.robot_id = robot_id
        self.text_display = RobotText(robot_id, pipeline)
        self.rendered_actors = None
        # self.model = None
        self.show_as_point = show_as_point
        self.colors = vtk_named_colors(
            ["Firebrick", "Gray", "Firebrick", "Firebrick", "Gray", "Firebrick", "Blue"]
        )
        self._create_robot(pipeline)

    def _create_robot(self, pipeline):
        logger.info(f"In helper function: {self.robot_id}")

        # base = np.matrix([[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 1.0], [0, 0, 0, 1]])

        # self.model = Inchworm(base=base)

        self._render(pipeline)
        logger.debug("Should be seeing new robot, as it was just added")

    def update(self, pipeline):
        # logger.info(f"Robot update called on robot: {self.robot_id}")
        if not self.robot_queue.empty():
            # logger.info(f"[{self.robot_id}]: Queue not empty")

            (
                robot_id,
                transforms,
                text_position,
                path,
                block_on_ee,
                debug_text,
            ) = self.robot_queue.get()
            for index in range(len(transforms)):
                assembly = vtk.vtkAssembly()
                """
                ROBOT
                """
                if not self.show_as_point:
                    if index == 6:
                        blockManager.update_block(
                            pipeline, transforms[index], block_on_ee
                        )

                    else:
                        if index == 0:
                            self.text_display.update(
                                pipeline, text_position, debug_text
                            )
                            # self.actors[0].SetUserMatrix(transforms[0])
                            # self.actors[0].SetScale(0.013)
                        self.actors[index].SetUserMatrix(transforms[index])
                        self.actors[index].SetScale(0.013)

                """
                POINT
                """
                if self.show_as_point:
                    assembly = self.actors[index]
                    assembly.SetUserMatrix(transforms[index])

            # pipeline.render()

    def _render(self, pipeline):
        # logger.info(f"In render: {self.robot_id}")

        if self.show_as_point:
            self._render_points_helper()
        else:
            self._render_robots_helper()
        # logger.info(f"For each actor, adding link")
        for link in self.actors:
            # pipeline.add_actor(link)
            pipeline.ren.AddActor(link)
        # logger.info(f"Finished adding links, now animating")

        # pipeline.render()

    def _render_points_helper(self):
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetCenter(0, 0, 0)
        sphereSource.SetRadius(0.3)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(
            uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
        )

        self.actors = [actor]

    def _render_robots_helper(self):
        stl_files = self._get_robot_files()
        print("STL FILES: {}".format(stl_files))
        self.reader_list = [0] * len(stl_files)
        self.actor_list = [0] * len(stl_files)
        self.mapper_list = [0] * len(stl_files)
        for i in range(len(stl_files)):
            self._render_helper(stl_files[i], i)

        # logger.info(f"Return from _render_helper: {self.actor_list}")
        self.actors = self.actor_list

    def _render_helper(
        self, file, index,
    ):
        # logger.info(f"In render helper: {self.robot_id}")

        self.reader_list[index] = vtk.vtkSTLReader()
        loc = pkg_resources.resource_filename(
            "components", "/".join(("simulator", "media", file))
        )
        self.reader_list[index].SetFileName(loc)
        self.mapper_list[index] = vtk.vtkPolyDataMapper()
        self.mapper_list[index].SetInputConnection(
            self.reader_list[index].GetOutputPort()
        )
        self.actor_list[index] = vtk.vtkActor()
        self.actor_list[index].SetMapper(self.mapper_list[index])
        self.actor_list[index].GetProperty().SetColor(self.colors[index])  # (R,G,B)
        self.actor_list[index].SetScale(0.013)

    def _get_robot_files(self, num=6):
        file_names = []
        for i in range(0, num):
            file_names.append("link" + str(i) + ".stl")
        print(f"NUMBER OF STL FILES: {file_names}")
        return file_names
