import vtk
import time
from itertools import cycle
from components.simulator.model.graphics import VtkPipeline


class Block:
    """
      This is the actor class for blocks. This class creates and updates a single block.

      """

    def __init__(self, pipeline, position, block_id, block_file_location, color):
        self.block_id = block_id
        self.showing = False
        self.color = color
        self._render(
            pipeline=pipeline,
            position=position,
            block_file_location=block_file_location,
            color=color,
        )
        self.color_start_time = None
        self.flicker_colors = cycle(
            [
                VtkPipeline.vtk_named_colors(["Yellow"]),
                VtkPipeline.vtk_named_colors(["Red"]),
            ]
        )
        self.color_flicker_frequency = 2.0

    def display(self, visibility):
        self.showing = visibility
        self.rendered_block.SetVisibility(self.showing)

    def update(
        self, pipeline, transform, color=VtkPipeline.vtk_named_colors(["Purple"])
    ):
        self.display(True)
        self.rendered_block.SetUserMatrix(transform)
        self.rendered_block.GetProperty().SetColor(color[0])
        self.rendered_block.SetScale(0.013)

    def flicker_color(self):
        if self.color_start_time is None:
            self.color_start_time = time.time()
        else:
            current_time = time.time()
            if current_time - self.color_start_time >= self.color_flicker_frequency:
                self.color = next(self.flicker_colors)
                self.rendered_block.GetProperty().SetColor(self.color[0])
                self.color_start_time = current_time

    def _render(self, pipeline, position, block_file_location, color):
        self.rendered_block = vtk.vtkActor()
        self.rendered_block.SetMapper(block_file_location)
        if color is None:
            self.color = VtkPipeline.vtk_named_colors(["Orange"])
        self.rendered_block.GetProperty().SetColor(self.color[0])  # (R,G,B)
        self.rendered_block.SetScale(0.013)
        self.display(False)

        if position:
            self.rendered_block.SetUserMatrix(position)
        pipeline.ren.AddActor(self.rendered_block)
