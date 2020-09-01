from random import uniform

import numpy as np
import pkg_resources
import vtk
from logzero import logger

from components.simulator.common.transforms import np2vtk
from components.simulator.entities.blocks.block_manager import blockManager
from components.simulator.entities.robots.robot_text import RobotText
from components.simulator.globals import *
from components.simulator.model.model import Inchworm


class Robot:
    """
    This is the actor class for robots. This class creates and updates a single robot. Currently, two types of robots
    are supported, one that shows the robots as an actual robot made from STL files, and another that shows the robots
    as dots. This can be toggled with the 'show_as_point' field.

    """

    def __init__(self, pipeline, robot_queue, robot_id, show_as_point=False):
        self.robot_queue = robot_queue
        self.robot_id = robot_id
        self.text_display = RobotText(robot_id, pipeline)
        self.rendered_actors = None
        self.show_as_point = show_as_point
        self.colors = VtkPipeline.vtk_named_colors(
            ["Firebrick", "Gray", "Firebrick", "Firebrick", "Gray", "Firebrick"]
        )
        self._create_robot(pipeline)

    def _get_updates(self):

        actor, message = self.robot_queue.get()

        """
        DISPLAY ROBOTS
        """
        if not self.show_as_point:
            # if isinstance(message.message, AnimationUpdateMessage): #TODO:
            base = message.message.robot_base
            trajectory = message.message.trajectory
            path = message.message.path
            block_on_ee = message.message.block_on_ee
            debug_text = message.message.debug_text
            robot = Inchworm(base=base)
            standing_on_block = True if block_on_ee else False
            transforms, robot_actors = robot.fkine(
                stance=trajectory,
                apply_stance=True,
                standing_on_block=standing_on_block,
                num_links=5,
            )

            text_position = np.eye(4)
            text_position[0, 3] = base[0, 3] + 1
            text_position[1, 3] = base[1, 3]
            text_position[2, 3] = base[2, 3] + 2
            text_position = np2vtk(text_position)

            return (
                robot_actors,
                text_position,
                path,
                block_on_ee,
                debug_text,
            )
        """
        DISPLAY POINTS
        """
        if self.show_as_point:
            new_position = message.message.robot_base
            transforms = np2vtk(new_position)

            path = message.message.path
            return ([transforms], None, path, None, None)

    def _create_robot(self, pipeline):
        logger.info(f"In helper function: {self.robot_id}")
        self._render(pipeline)
        logger.debug("Should be seeing new robot, as it was just added")

    def update(self, pipeline):
        if self.robot_queue.empty():
            return
        (transforms, text_position, path, block_on_ee, debug_text) = self._get_updates()
        for index in range(len(transforms)):
            assembly = vtk.vtkAssembly()
            """
            ROBOT
            """
            if not self.show_as_point:
                if index == 6:
                    blockManager.update_block(pipeline, transforms[index], block_on_ee)

                else:
                    if index == 0:
                        self.text_display.update(pipeline, text_position, debug_text)

                    self.actors[index].SetUserMatrix(transforms[index])
                    self.actors[index].SetScale(0.013)

            """
            POINT
            """
            if self.show_as_point:
                assembly = self.actors[index]
                assembly.SetUserMatrix(transforms[index])

    def _render(self, pipeline):
        if self.show_as_point:
            self._render_points_helper()
        else:
            self._render_robots_helper()
        for link in self.actors:
            pipeline.ren.AddActor(link)

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

        self.actors = self.actor_list

    def _render_helper(
        self, file, index,
    ):

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
        return file_names
