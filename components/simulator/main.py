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
from components.robot.communication.messages import BlockLocationMessage
from components.simulator.common.common import create_homogeneous_transform_from_point
from components.simulator.common.transforms import np2vtk
from components.simulator.model.create_actors import *
from components.simulator.model.graphics import *
from components.simulator.model.model import Inchworm
from components.simulator.events.gui_manager import GuiManager
from components.simulator.communication.communicate_with_robot import WorkerThread
from components.simulator.entities.orientation_widget import OrientationWidget

from swarm_c_library.blueprint_factory import BluePrintFactory

POINTS = False
BLUEPRINT = BluePrintFactory().get_blueprint("Playground").data

# BLUEPRINT = np.load("blueprint.npy")
bx, by, bz = BLUEPRINT.shape

COLORS = [[[vtk_named_colors(["DarkGreen"])] * bz] * by] * bx

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
manager = (
    GuiManager(pipeline=pipeline)
    .enable_keypress()
    .enable_color_highlighting()
    .enable_orientation_widget()
    .enable_sim_data_text()
)


class CalculatorThread(WorkerThread):
    """

    """

    def __init__(self, dir_q, result_q, filter_q, socket, pipeline, block_q):
        super(CalculatorThread, self).__init__(
            dir_q, result_q, filter_q, socket, pipeline, block_q
        )
        self.blocks = {}

    def run(self):
        """

        """
        while not self.stoprequest.isSet():
            if not self.dir_q.empty():
                actor, message = self.dir_q.get()

                """
                DISPLAY ROBOTS
                """
                if not POINTS:
                    # if isinstance(message.message, AnimationUpdateMessage): #TODO:
                    base = message.message.robot_base
                    trajectory = message.message.trajectory
                    path = message.message.path
                    block_on_ee = message.message.block_on_ee
                    debug_text = message.message.debug_text
                    robot = Inchworm(base=base)
                    standing_on_block = True if block_on_ee else False
                    transform, robot_actors = robot.fkine(
                        stance=trajectory,
                        apply_stance=True,
                        standing_on_block=standing_on_block,
                        num_links=5,
                    )

                    text_position = np.eye(4)
                    text_position[0, 3] = base[0, 3] + 1
                    text_position[1, 3] = base[1, 3]
                    text_position[2, 3] = base[2, 3] + 2
                    text_position = np2vtk(text_position)
                    self.result_q.put(
                        (
                            actor,
                            robot_actors,
                            text_position,
                            path,
                            block_on_ee,
                            debug_text,
                        )
                    )

                """
                DISPLAY POINTS
                """
                if POINTS:
                    new_position = message.message.robot_base
                    transform = np2vtk(new_position)

                    path = message.message.path
                    self.result_q.put((actor, [transform], None, path, None))


class vtkTimerCallback:
    """

    """

    def __init__(
        self,
        renderer,
        renderWindow,
        queue,
        new_actors,
        dir_q,
        result_q,
        socket,
        pipeline,
        block_q,
    ):
        self.timer_count = 0
        self.robot_actors = {}
        self.renderer = renderer
        self.renderWindow = renderWindow
        self.queue = queue
        self.new_actors = new_actors
        self.worker_pool = []
        self.dir_q = dir_q
        self.result_q = result_q
        self.socket = socket
        self.pipeline = pipeline
        self.colors = vtk_named_colors(
            ["Firebrick", "Gray", "Firebrick", "Firebrick", "Gray", "Firebrick"]
        )
        self.blocks = OrderedDict()
        self.block_q = block_q
        self.previous_path = []
        self.robot_texts = defaultdict(lambda: None)
        self.last_block_showing = None
        self.time_till_next_block = 100
        self.last_block_counter = self.time_till_next_block
        self.blocks_at_starting_location = []
        self.removed_starting_block = True

    def add_robot_to_sim(self, robot, result_queue):
        """

        :param robot:
        :param result_queue:
        """
        base = np.matrix([[1, 0, 0, 0.5], [0, 1, 0, 0.5], [0, 0, 1, 1.0], [0, 0, 0, 1]])

        new_robot = Inchworm(base=base, blueprint=BLUEPRINT)

        robot_actor, rendered_id = setup_pipeline_objs(
            colors=self.colors,
            robot_id=robot,
            points=POINTS,
            block_on_end_effector=False,
        )
        for link in robot_actor:
            self.pipeline.add_actor(link)

        if rendered_id:
            self.pipeline.add_actor(rendered_id)
            self.robot_texts[robot] = rendered_id

        logger.debug("Should be seeing new robot, as it was just added")

        self.robot_actors[robot] = (robot_actor, new_robot, result_queue, rendered_id)
        self.pipeline.animate()

    def create_new_thread(self, queue, result_queue):
        """

        :param queue:
        :param result_queue:
        """
        logger.debug("Callback: Creating new thread for robot")
        calculate_thread = CalculatorThread(
            dir_q=queue,
            result_q=result_queue,
            filter_q=self.new_actors,
            socket=self.socket,
            pipeline=self.pipeline,
            block_q=self.block_q,
        )
        self.worker_pool.append(calculate_thread)
        calculate_thread.daemon = True
        calculate_thread.start()

    def execute(self, obj, event):
        """

        :param obj:
        :param event:
        """
        manager.update_sim_data_text(len(self.robot_actors), len(self.blocks))

        if self.timer_count % 100 == 0:
            logger.debug(self.timer_count)

        self.timer_count += 1
        while not self.new_actors.empty():
            topic, message, queue = self.new_actors.get()
            result_q = Queue()
            logger.debug(f"Callback: Creating new thread for actor: {topic} {queue}")
            self.create_new_thread(queue, result_q)
            logger.debug(f"Callback: Adding new actor to sim: {topic}")
            self.add_robot_to_sim(topic, result_q)

        for robot in self.robot_actors:
            actors, model, robot_queue, rendered_id = self.robot_actors[robot]
            if not robot_queue.empty():
                (
                    robot_id,
                    transforms,
                    text_position,
                    path,
                    block_on_ee,
                    debug_text,
                ) = robot_queue.get()

                for index in range(len(transforms)):
                    assembly = vtk.vtkAssembly()
                    assembly.GetParts()
                    """
                    ROBOT
                    """
                    if not POINTS:
                        if index == 6:

                            if block_on_ee not in self.blocks:

                                ######## TOOL (CREATE) #############################
                                new_block_tool, _, _ = add_block(
                                    (0, 0, 0),
                                    block_file_location=move_block_file_location,
                                )
                                actors.append(new_block_tool)
                                new_block_tool.SetUserMatrix(transforms[index])
                                new_block_tool.SetScale(0.013)
                                new_block_tool.SetVisibility(True)
                                ######## TOOL (CREATE) #############################

                                self.pipeline.ren.AddActor(new_block_tool)

                                self.blocks[block_on_ee] = (
                                    transforms[index],
                                    new_block_tool,
                                    True,
                                )

                            else:

                                _, new_block_tool, _ = self.blocks[block_on_ee]

                                ######## TOOL (UPDATE) #############################
                                new_block_tool.SetUserMatrix(transforms[index])
                                color = vtk_named_colors(["Purple"])
                                new_block_tool.SetVisibility(True)
                                new_block_tool.GetProperty().SetColor(color[0])
                                self.blocks[block_on_ee] = (
                                    transforms[index],
                                    new_block_tool,
                                    True,
                                )
                                ######## TOOL (UPDATE) #############################

                                if block_on_ee in self.blocks_at_starting_location:
                                    self.blocks_at_starting_location.remove(block_on_ee)
                                    self.removed_starting_block = True

                        elif index <= 5:
                            if index == 0:
                                if self.robot_texts[robot_id]:
                                    rendered_id = self.robot_texts[robot_id]

                                ######## ROBOT TEXT #############################
                                if manager.show_debug_text():
                                    if debug_text:
                                        debug_text = debug_text.encode()
                                    else:
                                        debug_text = "".encode()
                                    text_for_robot = robot_id + debug_text
                                else:
                                    text_for_robot = robot_id
                                robot_text = vtk.vtkVectorText()
                                robot_text.SetText(text_for_robot)
                                robot_text_mapper = vtk.vtkPolyDataMapper()
                                robot_text_mapper.SetInputConnection(
                                    robot_text.GetOutputPort()
                                )
                                robot_text_actor = vtk.vtkActor()
                                robot_text_actor.SetMapper(robot_text_mapper)
                                robot_text_actor.GetProperty().SetColor(
                                    0.5, 0.5, 0.5,
                                )
                                robot_text_actor.AddPosition(0, 0, 1)
                                robot_text_actor.RotateX(60)
                                robot_text_actor.SetScale(0.5)
                                ######## ROBOT TEXT #############################

                                self.pipeline.remove_actor(rendered_id)
                                self.pipeline.add_actor(robot_text_actor)
                                self.pipeline.ren.AddActor(robot_text_actor)
                                self.robot_texts[robot_id] = robot_text_actor
                                self.robot_actors[robot] = (
                                    actors,
                                    model,
                                    robot_queue,
                                    robot_text_actor,
                                )
                                rendered_id = robot_text_actor

                                rendered_id.SetUserMatrix(text_position)
                            actors[index].SetUserMatrix(transforms[index])
                            actors[index].SetScale(0.013)

                    """
                    POINT
                    """
                    if POINTS:
                        assembly = actors[index]
                        assembly.SetUserMatrix(transforms[index])

                if path:

                    ######## PATH #############################
                    logger.debug(f"Path in Callback: {path}")
                    logger.debug(f"Previous path: {self.previous_path}")
                    for i in self.previous_path:
                        self.pipeline.remove_actor(i)
                        self.previous_path.remove(i)

                    for point in path:
                        self.previous_path.append(
                            self.pipeline.add_actor(cubeForPath(point))
                        )
                    ######## PATH #############################

                    self.pipeline.animate()

        while not self.block_q.empty():
            topic, message = self.block_q.get()
            if isinstance(message.message, BlockLocationMessage):
                if topic in self.blocks:
                    location, actor, showing = self.blocks[message.message.id]
                    actor.SetUserMatrix(
                        create_homogeneous_transform_from_point(
                            message.message.location
                        )
                    )
                    self.blocks[message.message.id] = (message.message.location, actor)
                else:
                    location = np.array(message.message.location)
                    if location[0] == 0.5 and location[1] == 0.5:
                        self.blocks_at_starting_location.append(message.message.id)
                    location[0] = float(location[0] + 0)
                    location[1] = float(location[1] + 0)
                    location[2] = float(location[2] + 0)
                    transform = np2vtk(
                        create_homogeneous_transform_from_point(location)
                    )

                    actor, _, _ = add_block(
                        (0, 0, 0),
                        block_file_location=move_block_file_location,
                        first_time=True,
                    )

                    actor.SetUserMatrix(transform)
                    actor.SetScale(0.013)
                    actor.SetVisibility(False)
                    self.blocks[message.message.id] = (transform, actor, False)
                    self.pipeline.add_actor(actor)
                    self.pipeline.animate()

        if len(self.blocks_at_starting_location) > 0:
            if self.removed_starting_block:
                self.last_block_counter += 1
                if self.last_block_counter >= self.time_till_next_block:
                    block = self.blocks_at_starting_location[-1]
                    transform, actor, showing = self.blocks[block]
                    if not showing:
                        actor.SetVisibility(True)
                        self.blocks[block] = (transform, actor, True)
                    self.removed_starting_block = False
                    self.last_block_counter = 0
        iren = obj
        iren.GetRenderWindow().Render()


class Simulate:
    """

    """

    def __init__(self, robot_update, dir_q, result_q, new_actors, socket, block_q):
        self.robot_actors = {}
        self.worker_pool = []
        self.robot_update = robot_update
        self.dir_q = dir_q
        self.result_q = result_q
        self.new_actors = new_actors
        self.socket = socket
        self.block_q = block_q

    def wait_for_structure_initialization(self, blueprint=None, colors=None):
        """

        :param blueprint:
        :param colors:
        """
        global BLUEPRINT
        global COLORS
        if blueprint is not None:
            BLUEPRINT = blueprint
        if colors is not None:
            COLORS = colors
        else:
            [topic, message] = self.socket.recv_multipart()
            message = zlib.decompress(message)
            messagedata = pickle.loads(message)
            logger.debug(f"[Worker thread]: {topic} {messagedata}")
            if topic == b"STRUCTURE":
                BLUEPRINT = messagedata.message.blueprint
                COLORS = messagedata.message.colors
                print(BLUEPRINT)
                print(COLORS)
        self.simulate()

    def simulate(self, pipeline):
        """

        """
        # robot_actors = {}

        self.pipeline = pipeline
        self.param = {
            "cube_axes_x_bounds": np.array([[0, len(BLUEPRINT)]]),
            "cube_axes_y_bounds": np.array([[0, len(BLUEPRINT[0])]]),
            "cube_axes_z_bounds": np.array([[0, len(BLUEPRINT[0][0])]]),
            "floor_position": np.array([[0, 0, 0]]),
        }

        cube_axes = axesCubeFloor(
            self.pipeline.ren,
            self.param.get("cube_axes_x_bounds"),
            self.param.get("cube_axes_y_bounds"),
            self.param.get("cube_axes_z_bounds"),
            self.param.get("floor_position"),
        )

        self.pipeline.add_actor(cube_axes)

        cb = vtkTimerCallback(
            new_actors=self.new_actors,
            renderer=self.pipeline.ren,
            renderWindow=self.pipeline.ren_win,
            queue=self.robot_update,
            dir_q=self.dir_q,
            result_q=self.result_q,
            socket=self.socket,
            pipeline=self.pipeline,
            block_q=self.block_q,
        )

        self.pipeline.iren.AddObserver("TimerEvent", cb.execute)
        self.pipeline.iren.AddObserver(
            "LeftButtonPressEvent", manager.leftButtonPressEvent
        )
        self.pipeline.iren.AddObserver("KeyPressEvent", manager.keypress)

        self.pipeline.iren.CreateRepeatingTimer(10)

        setup_structure_display(
            blueprint=BLUEPRINT,
            pipeline=self.pipeline,
            color=COLORS,
            block_file_location=block_file_location,
        )

        self.pool = [
            WorkerThread(
                dir_q=self.dir_q,
                result_q=self.result_q,
                filter_q=self.new_actors,
                socket=socket,
                pipeline=self.pipeline,
                block_q=self.block_q,
            )
        ]

        # Start all threads
        for thread in self.pool:
            thread.daemon = True
            thread.start()
            logger.info("Started worker thread")

        try:
            self.pipeline.animate()
        except KeyboardInterrupt:
            logger.info("Exiting")


if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    logger.info("Collecting updates from simulator...")
    socket.bind(Config.communication["receive_messages_port"])

    socket.setsockopt(zmq.SUBSCRIBE, b"")

    dir_q = Queue()
    result_q = Queue()
    new_actors = Queue()
    robot_update = Queue()
    block_queue = Queue()

    sim = Simulate(
        robot_update=robot_update,
        dir_q=dir_q,
        result_q=result_q,
        new_actors=new_actors,
        socket=socket,
        block_q=block_queue,
    )
    sim.simulate(pipeline=pipeline)
    # sim.wait_for_structure_initialization(blueprint=BLUEPRINT, colors=COLORS)
    # sim.wait_for_structure_initialization()
