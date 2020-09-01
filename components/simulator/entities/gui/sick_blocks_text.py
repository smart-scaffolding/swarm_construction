import os
import time
from csv import DictWriter
from random import uniform

import vtk

import components.simulator.config as Config


class SickBlocksText:
    """
    This class controls the text that is displayed in the simulator. It should be used to communicate useful
    information such as the total number of time steps in the current simulation.

    """

    def __init__(self, iren, color=None):

        self._create_widget(color)
        self._set_widget_properties()
        self._enable_widget(iren)

    def update_text(self, sick_blocks):
        output_text = f"Blocks that need to be replaced: \n--------------------------------------------\n\n"
        for block in sick_blocks:
            output_text += f"\t {block.decode()}\n"

        self.text_actor.SetInput(output_text)
        self._show_widget()

    def _create_widget(self, color):
        self.text_actor = vtk.vtkTextActor()
        self.text_actor.SetInput(f"Blocks that need to be replaced: ")
        if color:
            self.text_actor.GetTextProperty().SetColor(color)
        else:
            self.text_actor.GetTextProperty().SetColor(1, 0, 0)
        self.text_actor.GetTextProperty().SetFontSize(100)

    def _set_widget_properties(self):
        self.text_representation = vtk.vtkTextRepresentation()
        self.text_representation.GetPosition2Coordinate().SetValue(0.3, 1.7)
        self.text_representation.SetPosition(0.68, 0.06)
        self.text_representation.SetWindowLocation(
            self.text_representation.UpperRightCorner
        )

    def _enable_widget(self, iren):
        self.text_widget = vtk.vtkTextWidget()
        self.text_widget.SetRepresentation(self.text_representation)
        self.text_widget.SetInteractor(iren)
        self.text_widget.SetTextActor(self.text_actor)
        self.text_widget.SelectableOff()

    def _show_widget(self, on=True):
        if on:
            self.text_widget.On()
        else:
            self.text_widget.Off()

    def hide(self):
        self._show_widget(on=False)
