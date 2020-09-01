import pickle
import zlib

import numpy as np
from logzero import logger

import components.simulator.config as Config
from components.simulator.communication.communicate_with_robot import (
    SimulatorCommunicator,
)
from components.simulator.entities.blocks.block_manager import blockManager
from components.simulator.entities.gui.axis_entity import AxisFactory
from components.simulator.entities.robots.robot_manager import robotManager
from components.simulator.globals import (
    robot_queue,
    block_queue,
    structure_queue,
    pipeline,
    guiManager,
)


class EventLoop:
    """
    This class is the callback that is called for every time step of the event loop. This class should be used to
    update all entities in the simulation for each time step.

    """

    def __init__(
        self, pipeline,
    ):
        self.timer_count = 0
        self.pipeline = pipeline

    def execute(self, obj, event):
        """
        Updates all entities in simulation (GUI, robots, blocks) for each time step

        """

        # Updates text module with info such as number of timesteps
        guiManager.update_sim_data_text(
            robotManager.get_robot_count(),
            blockManager.get_block_count(),
            self.timer_count,
            blockManager.get_blocks_in_env(),
        )

        guiManager.update_sick_blocks_text(blockManager.get_sick_blocks())

        # If the simulation has finished (must receive message from structure that all divisions have been built),
        # the simulator will save the results to a file
        guiManager.check_for_simulation_finish(
            structure_queue,
            robotManager.get_robot_count(),
            blockManager.get_block_count(),
            self.timer_count,
            blockManager.get_blocks_in_env(),
        )

        if self.timer_count % Config.PRINT_TIMER_FREQUENCY == 0:
            logger.debug(self.timer_count)
        self.timer_count += 1

        # Update all actors in module
        robotManager.update(self.pipeline)
        blockManager.update(self.pipeline)

        iren = obj
        iren.GetRenderWindow().Render()


class Simulate:
    """
    This class is the entry point to the simulation and should be used to create the desired simulator.

    """

    def __init__(self, robot_q, block_q, structure_q):
        self.robot_q = robot_q
        self.block_q = block_q
        self.structure_q = structure_q

    def wait_for_structure_initialization(self):
        """
        Simulator will wait for message from structure in order to start. 
        NOTE: This function expects a message that includes a blueprint and its colors before starting the simulation. 

        TODO: Add imporoved error checking. UNSAFE FUNCTION
        """

        while True:
            if not self.structure_q.empty():
                [topic, message] = self.structure_q.get()
                message = zlib.decompress(message)
                messagedata = pickle.loads(message)
                Config.BLUEPRINT = messagedata.message.blueprint
                Config.COLORS = messagedata.message.colors
                self.simulate(pipeline)

    def simulate(self, pipeline):
        """
        Method used to actual start the simulator
        """

        self.pipeline = pipeline

        # Create axes for simulator
        self.param = {
            "cube_axes_x_bounds": np.array([[0, len(Config.BLUEPRINT)]]),
            "cube_axes_y_bounds": np.array([[0, len(Config.BLUEPRINT[0])]]),
            "cube_axes_z_bounds": np.array([[0, len(Config.BLUEPRINT[0][0])]]),
        }
        cube_axes = AxisFactory.axes_cube_floor(
            self.pipeline.ren,
            self.param.get("cube_axes_x_bounds"),
            self.param.get("cube_axes_y_bounds"),
            self.param.get("cube_axes_z_bounds"),
        )
        self.pipeline.add_actor(cube_axes)

        # Initialize callbacks
        cb = EventLoop(pipeline=self.pipeline)
        self.pipeline.iren.AddObserver("TimerEvent", cb.execute)
        self.pipeline.iren.AddObserver(
            "LeftButtonPressEvent", guiManager.leftButtonPressEvent
        )
        self.pipeline.iren.AddObserver("KeyPressEvent", guiManager.keypress)
        self.pipeline.iren.CreateRepeatingTimer(10)

        # Create existing structure
        blockManager.setup_structure_display(
            blueprint=Config.BLUEPRINT, pipeline=self.pipeline, color=Config.COLORS,
        )

        # Start all communicators
        self.pool = [
            SimulatorCommunicator(
                robot_q=self.robot_q,
                block_q=self.block_q,
                structure_q=self.structure_q,
            )
        ]

        # Start all threads
        for thread in self.pool:
            thread.daemon = True
            thread.start()
            logger.info("Started communication thread")

        # Start simulation
        try:
            logger.info("Simulation started")
            self.pipeline.animate()
        except KeyboardInterrupt:
            logger.info("Exiting")


if __name__ == "__main__":

    logger.info("Starting up simulator...")

    sim = Simulate(
        robot_q=robot_queue, block_q=block_queue, structure_q=structure_queue,
    )
    sim.simulate(pipeline=pipeline)
    # sim.wait_for_structure_initialization()
