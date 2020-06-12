from __future__ import print_function

import pickle
import threading
import time
import zlib
from collections import defaultdict, OrderedDict
from queue import Queue

import zmq
from logzero import logger

import components.simulator.config as Config

from components.simulator.model.create_actors import *
from components.simulator.model.graphics import *
from components.simulator.model.model import Inchworm
from components.simulator.events.gui_manager import GuiManager
from components.simulator.entities.block_manager import BlockManager

from swarm_c_library.blueprint_factory import BluePrintFactory

loc = pkg_resources.resource_filename(
    "components", "/".join(("simulator", "media", "block.stl"))
)
reader_list = vtk.vtkSTLReader()
reader_list.SetFileName(loc)
reader_list.GetOutput().GlobalReleaseDataFlagOn()
block_file_location = vtk.vtkPolyDataMapper()
block_file_location.SetInputConnection(reader_list.GetOutputPort())

move_block_loc = pkg_resources.resource_filename(
    "components", "/".join(("simulator", "media", "robot_block.stl"))
)
reader_list = vtk.vtkSTLReader()
reader_list.SetFileName(move_block_loc)
move_block_file_location = vtk.vtkPolyDataMapper()
move_block_file_location.SetInputConnection(reader_list.GetOutputPort())

pipeline = VtkPipeline(gif_file=None)
guiManager = (
    GuiManager(pipeline=pipeline)
    .enable_keypress()
    .enable_color_highlighting()
    .enable_orientation_widget()
    .enable_sim_data_text()
)

dir_q = Queue()
result_q = Queue()
new_actors = Queue()
robot_update = Queue()
block_queue = Queue()

blockManager = BlockManager(block_queue)
