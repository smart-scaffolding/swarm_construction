from collections import OrderedDict

import numpy as np
import pkg_resources
import vtk

from components.robot.communication.messages import BlockLocationMessage
from components.simulator.common.common import create_homogeneous_transform_from_point
from components.simulator.common.transforms import np2vtk
from components.simulator.entities.blocks.block_entity import Block
from components.simulator.globals import block_queue
from swarm_c_library.profiling.debug import timefunc
from components.simulator.model.graphics import VtkPipeline

import components.simulator.config as Config


class BlockManager:
    """
    Manager class for all block entities. It calls each block to update (change their position/color), as well as deals
    with the creation and removal of all block entities.
    
    """

    def __init__(self, block_q):
        self.blocks = OrderedDict()
        self.block_q = block_q

        self.last_block_showing = None
        self.time_till_next_block = 200
        self.last_block_counter = self.time_till_next_block
        self.blocks_at_starting_location = []
        self.removed_starting_block = True
        self.blocks_already_in_env = 0
        self.colored_blocks = []

        self.feeding_locations = Config.FEEDING_LOCATIONS
        self.use_feeding_locations = Config.USE_FEEDING_LOCATION

        loc = pkg_resources.resource_filename(
            "components", "/".join(("simulator", "media", "block.stl"))
        )
        reader_list = vtk.vtkSTLReader()
        reader_list.SetFileName(loc)
        reader_list.GetOutput().GlobalReleaseDataFlagOn()
        self.block_file_location = vtk.vtkPolyDataMapper()
        self.block_file_location.SetInputConnection(reader_list.GetOutputPort())

    @timefunc("BlockManagerUpdate")
    def update(self, pipeline):
        while not self.block_q.empty():
            topic, message = self.block_q.get()
            if isinstance(message.message, BlockLocationMessage):
                block_id = message.message.id
                location = message.message.location
                status = message.message.status
                color = message.message.color
                if color is not None:
                    color = VtkPipeline.vtk_named_colors([color])
                if status == "-":
                    self.remove_state(block_id, pipeline)
                elif status == "?":
                    self.sick_state(block_id, topic, location, pipeline)
                else:
                    self.add_state(block_id, topic, location, pipeline, color=color)
        self.flicker_blocks()

        self.check_if_human_placed_block()

    def update_block(self, pipeline, transform, block_id):
        if block_id in self.blocks_at_starting_location:
            self.blocks_at_starting_location.remove(block_id)
            self.removed_starting_block = True
        if block_id in self.blocks:
            self.blocks[block_id].update(pipeline, transform)
        else:
            self.add_new_block(pipeline, transform, block_id)

    def _calc_transform(self, location, block_id, use_feeding_location=True):
        location = np.array(location)
        at_feeding_location = False
        if self._check_if_feeding_location(location) and use_feeding_location:
            self.blocks_at_starting_location.append(block_id)
            at_feeding_location = True
        transform = np2vtk(create_homogeneous_transform_from_point(location))
        return transform, at_feeding_location

    def _check_if_feeding_location(self, block_location):
        for locations in self.feeding_locations:
            if block_location[0] == locations[0] and block_location[1] == locations[0]:
                return True
        return False

    def add_state(self, block_id, topic, location, pipeline, show=True, color=None):
        transform, at_feeding_location = self._calc_transform(
            location, block_id, self.use_feeding_locations
        )
        if topic in self.blocks:
            self.blocks[topic].update(pipeline, transform, show=show)
        else:
            self.add_new_block(
                pipeline,
                transform,
                block_id,
                display=(not at_feeding_location),
                color=color,
            )
        return topic

    def remove_state(self, block_id, pipeline):
        if block_id in self.blocks:
            block = self.blocks.pop(block_id)
            if block_id in self.colored_blocks:
                self.colored_blocks.remove(block_id)
            block.remove(pipeline)

    def sick_state(self, block_id, topic, location, pipeline):
        block_id = self.add_state(block_id, topic, location, pipeline, show=False)
        if block_id not in self.colored_blocks:
            self.colored_blocks.append(block_id)

    def add_new_block(self, pipeline, transform, block_id, color=None, display=False):
        self.blocks[block_id] = Block(
            pipeline, transform, block_id, self.block_file_location, color, display
        )
        return self.blocks[block_id]

    def flicker_blocks(self):
        for block_id in self.colored_blocks:
            self.blocks[block_id].flicker_color()

    def check_if_human_placed_block(self):
        if len(self.blocks_at_starting_location) > 0:
            if self.removed_starting_block:
                self.last_block_counter += 1
                if self.last_block_counter >= self.time_till_next_block:
                    block = self.blocks_at_starting_location[0]
                    self.blocks[block].display(True)
                    self.removed_starting_block = False
                    self.last_block_counter = 0

    def setup_structure_display(self, blueprint, pipeline, color):

        """
        Internal function to initialise vtk objects.
        :return: reader_list, actor_list, mapper_list
        """
        for i in range(len(blueprint)):
            for j in range(len(blueprint[0])):
                for k in range(len(blueprint[0][0])):
                    if blueprint[i][j][k]:
                        block_id = f"BLOCK_{i}_{j}_{k}"
                        transform, _ = self._calc_transform(
                            (i, j, k), block_id, use_feeding_location=False
                        )
                        my_color = color[i][j][k]
                        self.add_new_block(pipeline, transform, block_id, my_color)
                        self.blocks[block_id].display(True)
                        self.blocks_already_in_env += 1

    def get_block_count(self):
        return len(self.blocks.keys()) - self.blocks_already_in_env

    def get_blocks_in_env(self):
        return self.blocks_already_in_env

    def get_sick_blocks(self):
        return self.colored_blocks


# NOTE: The manager is defined here
blockManager = BlockManager(block_queue)
