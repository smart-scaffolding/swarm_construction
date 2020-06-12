from collections import OrderedDict
from components.simulator.globals import *
from components.robot.communication.messages import BlockLocationMessage
from components.simulator.model.graphics import vtk_named_colors
from components.simulator.common.common import create_homogeneous_transform_from_point
from components.simulator.common.transforms import np2vtk
import numpy as np

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


class BlockManager:
    def __init__(self, block_q):
        self.blocks = OrderedDict()
        self.block_q = block_q

        self.last_block_showing = None
        self.time_till_next_block = 200
        self.last_block_counter = self.time_till_next_block
        self.blocks_at_starting_location = []
        self.removed_starting_block = True

    def update(self, pipeline):
        while not self.block_q.empty():
            logger.info(f"NOT EMPTY: Calling update on block manager")

            topic, message = self.block_q.get()
            if isinstance(message.message, BlockLocationMessage):
                block_id = message.message.id
                location = message.message.location
                transform = self._calc_transform(location, block_id)
                if topic in self.blocks:
                    logger.info(f"Updating block: {topic}")
                    # if block_id in self.blocks_at_starting_location:
                    #     # self.blocks_at_starting_location.remove(block_id)
                    #     self.removed_starting_block = True

                    self.blocks[topic].update(pipeline, transform)
                else:
                    logger.info(f"Adding new block: {topic}")

                    self.add_new_block(pipeline, transform, block_id)

    def update_block(self, pipeline, transform, block_id):
        if block_id in self.blocks_at_starting_location:
            self.blocks_at_starting_location.remove(block_id)
            logger.warning(f"BLOCK HAS BEEN REMOVED, SETTING TO TRUE ----> {block_id}")
            self.removed_starting_block = True
        else:
            logger.info(
                f"BLOCK NOT IN LIST {block_id} ----> {self.blocks_at_starting_location}"
            )
            # self.removed_starting_block = True
        if block_id in self.blocks:
            self.blocks[block_id].update(pipeline, transform)
        else:
            self.add_new_block(pipeline, transform, block_id)

    def _calc_transform(self, location, block_id):
        location = np.array(location)
        if location[0] == 0 and location[1] == 0:
            logger.info(f"BLOCK AT STARTING LOCATION: {block_id}")
            self.blocks_at_starting_location.append(block_id)
        # location[0] = float(location[0] + 0)
        # location[1] = float(location[1] + 0)
        # location[2] = float(location[2] + 0)
        transform = np2vtk(create_homogeneous_transform_from_point(location))
        return transform

    def add_new_block(self, pipeline, transform, block_id):
        self.blocks[block_id] = Block(pipeline, transform, block_id)

    def check_if_human_placed_block(self):
        if len(self.blocks_at_starting_location) > 0:
            # print(
            #     f"{len(self.blocks_at_starting_location)} blocks at starting location"
            # )
            if self.removed_starting_block:
                self.last_block_counter += 1
                print(f"Counter: {self.last_block_counter}")
                if self.last_block_counter >= self.time_till_next_block:
                    block = self.blocks_at_starting_location[0]
                    print(f"Now displaying block: {block}")
                    self.blocks[block].display(True)
                    self.removed_starting_block = False
                    # self.blocks_at_starting_location.remove(block)
                    self.last_block_counter = 0


class Block:
    def __init__(self, pipeline, position, block_id):
        self.block_id = block_id
        self.showing = False
        self._render(pipeline=pipeline, position=position)

    def display(self, visibility):
        self.showing = visibility
        self.rendered_block.SetVisibility(self.showing)

    def update(self, pipeline, transform, color=vtk_named_colors(["Purple"])):
        self.display(True)
        self.rendered_block.SetUserMatrix(transform)
        self.rendered_block.GetProperty().SetColor(color[0])
        self.rendered_block.SetScale(0.013)

    def _render(self, pipeline, position):
        self.rendered_block = vtk.vtkActor()
        self.rendered_block.SetMapper(block_file_location)
        if not self.showing:
            color = vtk_named_colors(["Orange"])
        else:
            color = vtk_named_colors(["Purple"])

        self.rendered_block.GetProperty().SetColor(color[0])  # (R,G,B)
        self.rendered_block.SetScale(0.013)
        self.display(False)

        if position:
            self.rendered_block.SetUserMatrix(position)
        # pipeline.add_actor(self.rendered_block)
        pipeline.ren.AddActor(self.rendered_block)
        # pipeline.animate()
