import vtk
import time
from random import uniform


class SimDataText:
    def __init__(self, iren, color=None):

        self.start_time = time.time()
        self._create_widget(color)
        self._set_widget_properties()
        self._show_widget(iren)

    def update_text(self, num_robots, num_blocks):
        elapsed_time = time.time() - self.start_time

        self.text_actor.SetInput(
            f"Simulation Time: {time.strftime('%H:%M:%S', time.gmtime(elapsed_time))}\nNumber of "
            f"Robots: {num_robots}\nNumber of Blocks Placed: {num_blocks}"
        )

    def _create_widget(self, color):
        self.text_actor = vtk.vtkTextActor()
        self.text_actor.SetInput(
            f"Simulation Time: 00:00:00\nNumber of Robots: 0\nNumber of Blocks Placed: 0"
        )
        if color:
            self.text_actor.GetTextProperty().SetColor(color)
        else:
            self.text_actor.GetTextProperty().SetColor(
                uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
            )

    def _set_widget_properties(self):
        self.text_representation = vtk.vtkTextRepresentation()
        self.text_representation.GetPosition2Coordinate().SetValue(0.2, 1.8)
        self.text_representation.GetSize([2, 0.5])
        self.text_representation.SetWindowLocation(
            self.text_representation.UpperLeftCorner
        )

    def _show_widget(self, iren):
        self.text_widget = vtk.vtkTextWidget()
        self.text_widget.SetRepresentation(self.text_representation)
        self.text_widget.SetInteractor(iren)
        self.text_widget.SetTextActor(self.text_actor)
        self.text_widget.SelectableOff()
        self.text_widget.On()
