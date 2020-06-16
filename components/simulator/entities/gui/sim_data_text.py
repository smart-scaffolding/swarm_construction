import os
import time
from csv import DictWriter
from random import uniform

import vtk

import components.simulator.config as Config


class SimDataText:
    """
    This class controls the text that is displayed in the simulator. It should be used to communicate useful
    information such as the total number of time steps in the current simulation.

    """
    def __init__(self, iren, color=None):

        self.start_time = time.time()
        self._create_widget(color)
        self._set_widget_properties()
        self._show_widget(iren)

    def update_text(self, num_robots, num_blocks, num_timesteps, base_num_blocks):
        elapsed_time = time.time() - self.start_time

        self.text_actor.SetInput(
            f"Simulation Time: {time.strftime('%H:%M:%S', time.gmtime(elapsed_time))}\nNumber of "
            f"Robots: {num_robots}\nNumber of Blocks Placed: {num_blocks}\n Number of Time steps: {num_timesteps}"
            f"\nBase Number of Blocks: {base_num_blocks}"
        )

    def save_results(
        self, filename, num_robots, num_blocks, num_timesteps, base_num_blocks
    ):
        """
        If enabled, this method can be used to save the metrics of the simulation (number of timesteps,
        number of blocks, number of robots, and total simulation time) to a csv. Furthermore, if notifications are
        enabled (for Mac users only), a notification will be sent once this has been triggered to signify the end of
        the simulation.

        """
        elapsed_time = time.time() - self.start_time
        dict_to_write = {
            "Number of timesteps": num_timesteps,
            "Number of blocks": num_blocks,
            "Number of robots": num_robots,
            "Simulation time": time.strftime("%H:%M:%S", time.gmtime(elapsed_time)),
        }
        with open(filename, "a", newline="") as csvfile:
            fieldnames = dict_to_write.keys()
            writer = DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(dict_to_write)

        if Config.NOTIFY_WHEN_SIMULATION_FINISHED:
            for _ in range(5):
                os.system(
                    """
                    osascript -e 'display notification "Done simulating" with title "Finished" sound name "Glass"'
                    """
                )
                time.sleep(1)

    def _create_widget(self, color):
        self.text_actor = vtk.vtkTextActor()
        self.text_actor.SetInput(
            f"Simulation Time: 00:00:00\nNumber of Robots: 0\nNumber of Blocks Placed: 0\nNumber of Time steps: 0\nBase "
            f"Number of Blocks: 0"
        )
        if color:
            self.text_actor.GetTextProperty().SetColor(color)
        else:
            self.text_actor.GetTextProperty().SetColor(
                uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
            )

    def _set_widget_properties(self):
        self.text_representation = vtk.vtkTextRepresentation()
        self.text_representation.GetPosition2Coordinate().SetValue(0.2, 1.7)
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
