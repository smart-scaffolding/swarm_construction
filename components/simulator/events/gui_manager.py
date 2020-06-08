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

    def enable_keypress(self):
        self.keypress_enabled = True
        self.keypress_command = Keypress()
        return self

    def enable_color_highlighting(self):
        self.color_selection_enabled = True
        self.selector_command = Selector(pipeline=self.pipeline)
        return self

    def keypress(self, obj, event):
        if self.keypress_enabled:
            self.keypress_command.execute(obj, event)

    def leftButtonPressEvent(self, obj, event):
        if self.color_selection_enabled:
            self.selector_command.execute(obj, event)

    def show_debug_text(self):
        if self.keypress_enabled:
            return self.keypress_command.DEBUG
