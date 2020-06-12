from components.simulator.globals import *
import vtk


class RobotText:
    def __init__(self, robot_id, pipeline):
        self.robot_id = robot_id
        self._render_text(text_for_robot=robot_id, pipeline=pipeline)

    def _render_text(self, text_for_robot, pipeline):
        robot_text = vtk.vtkVectorText()
        robot_text.SetText(text_for_robot)
        robot_text_mapper = vtk.vtkPolyDataMapper()
        robot_text_mapper.SetInputConnection(robot_text.GetOutputPort())
        self.robot_text_actor = vtk.vtkActor()
        self.robot_text_actor.SetMapper(robot_text_mapper)
        self.robot_text_actor.GetProperty().SetColor(
            0.5, 0.5, 0.5,
        )
        self.robot_text_actor.AddPosition(0, 0, 1)
        self.robot_text_actor.RotateX(60)
        self.robot_text_actor.SetScale(0.5)
        pipeline.add_actor(self.robot_text_actor)

    def update(self, pipeline, text_position, debug_text):

        if guiManager.show_debug_text():
            if debug_text:
                debug_text = debug_text.encode()
            else:
                debug_text = "".encode()
            text_for_robot = self.robot_id + debug_text
        else:
            text_for_robot = self.robot_id

        pipeline.remove_actor(self.robot_text_actor)

        self._render_text(text_for_robot, pipeline)
        pipeline.add_actor(self.robot_text_actor)
        pipeline.ren.AddActor(self.robot_text_actor)
        self.robot_text_actor.SetUserMatrix(text_position)
