from queue import Queue

from components.simulator.events.gui_manager import GuiManager
from components.simulator.model.graphics import VtkPipeline

"""
This module defines entities that should be kept as globals so that they can be accessed across other modules. These 
include the pipeline used for rendering, the GUI manager which can be updated by different components, and queues 
that should be used to pass messages between threads. 

"""

pipeline = VtkPipeline(gif_file=None)
guiManager = (
    GuiManager(pipeline=pipeline)
    .enable_keypress()
    .enable_color_highlighting()
    .enable_orientation_widget()
    .enable_sim_data_text()
    .enable_sick_blocks_text()
)

robot_queue = Queue()
block_queue = Queue()
structure_queue = Queue()
