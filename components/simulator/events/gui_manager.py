from components.simulator.entities.gui.orientation_widget import OrientationWidget
from components.simulator.entities.gui.sim_data_text import SimDataText
from components.simulator.events.keypress import Keypress
from components.simulator.events.selector import Selector


class GuiManager:
    """
    Manages all events related to GUI, such as keypresses. This uses the builder pattern to set which events are enabled
    """

    def __init__(self, pipeline):
        self.keypress_enabled = False
        self.color_selection_enabled = False
        self.pipeline = pipeline

        self.keypress_command = None
        self.selector_command = None
        self.orientation_widget = None
        self.sim_data_text = None

    def enable_keypress(self):
        self.keypress_enabled = True
        self.keypress_command = Keypress()
        return self

    def enable_color_highlighting(self):
        self.color_selection_enabled = True
        self.selector_command = Selector(pipeline=self.pipeline)
        return self

    def enable_orientation_widget(self, xyzLabels=("X", "Y", "Z"), scale=(1.0, 1.0, 1.0)):
        self.orientation_widget = OrientationWidget(
            iren=self.pipeline.iren, xyzLabels=xyzLabels, scale=scale
        )
        return self

    def enable_sim_data_text(self, color=None):
        self.sim_data_text = SimDataText(self.pipeline.iren, color)
        return self

    def update_sim_data_text(
        self, num_robots, num_blocks, num_timesteps, base_num_blocks
    ):
        if self.sim_data_text:
            self.sim_data_text.update_text(
                num_robots, num_blocks, num_timesteps, base_num_blocks
            )

    def check_for_simulation_finish(
        self, structure_queue, num_robots, num_blocks, num_timesteps, base_num_blocks
    ):
        while not structure_queue.empty():
            _, message = structure_queue.get()
            filename = message.message.filename
            self.sim_data_text.save_results(
                filename, num_robots, num_blocks, num_timesteps, base_num_blocks
            )

    def keypress(self, obj, event):
        if self.keypress_enabled:
            self.keypress_command.execute(obj, event)

    def leftButtonPressEvent(self, obj, event):
        if self.color_selection_enabled:
            self.selector_command.execute(obj, event)

    def show_debug_text(self):
        if self.keypress_enabled:
            return self.keypress_command.DEBUG
