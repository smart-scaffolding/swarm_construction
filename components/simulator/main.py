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
from swarm_c_library.blueprint_factory import BluePrintFactory

POINTS = False
ROBOTS = 1
DEBUG = False
DEBUG_TOGGLED = False
BLUEPRINT = BluePrintFactory().get_blueprint("StairwayToHeaven").data


# BLUEPRINT = np.load("blueprint.npy")
bx, by, bz = BLUEPRINT.shape

COLORS = np.array([[[vtk_named_colors(["DarkGreen"])] * bz] * by] * bx)

loc = pkg_resources.resource_filename(
    "components", "/".join(("simulator", "media", "block.stl"))
)
reader_list = vtk.vtkSTLReader()
reader_list.SetFileName(loc)
block_file_location = vtk.vtkPolyDataMapper()
block_file_location.SetInputConnection(reader_list.GetOutputPort())

move_block_loc = pkg_resources.resource_filename(
    "components", "/".join(("simulator", "media", "robot_block.stl"))
)
reader_list = vtk.vtkSTLReader()
reader_list.SetFileName(move_block_loc)
move_block_file_location = vtk.vtkPolyDataMapper()
move_block_file_location.SetInputConnection(reader_list.GetOutputPort())

text_actor = vtk.vtkTextActor()
text_actor.SetInput(
    f"Simulation Time: 00:00:00\nNumber of Robots: 0\nNumber of Blocks Placed: 0"
)
text_actor.GetTextProperty().SetColor(
    uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
)

text_representation = vtk.vtkTextRepresentation()
text_representation.GetPosition2Coordinate().SetValue(0.2, 1.8)
text_representation.GetSize([2, 0.5])
text_representation.SetWindowLocation(text_representation.UpperLeftCorner)
start_time = time.time()


class WorkerThread(threading.Thread):
    """

    """

    def __init__(self, dir_q, result_q, filter_q, socket, pipeline, block_q):
        super(WorkerThread, self).__init__()
        self.robot_actors = {}
        self.dir_q = dir_q
        self.result_q = result_q
        self.filter_q = filter_q
        self.stoprequest = threading.Event()
        self.socket = socket
        self.pipeline = pipeline
        self.block_q = block_q

    def run(self):
        """

        """
        while not self.stoprequest.isSet():
            try:
                [topic, message] = self.socket.recv_multipart()
                message = zlib.decompress(message)
                messagedata = pickle.loads(message)
                logger.debug(f"[Worker thread]: {topic} {messagedata}")
                if "BLOCK" in str(topic.decode()):
                    print(f"[Worker thread]: Got block message: {topic} -> {messagedata}")
                    self.block_q.put((topic, messagedata))
                if "ROBOT" in str(topic.decode()):
                    if topic not in self.robot_actors:
                        # print("[Worker thread]: Received new robot connection, adding to queue")
                        self.robot_actors[topic] = Queue()
                        self.robot_actors[topic].put((topic, messagedata))
                        self.filter_q.put(
                            (topic, messagedata, self.robot_actors[topic])
                        )  # add new robot
                        # create new worker here

                        # self.result_q.put((topic, messagedata))
                    else:
                        # print(f"[Worker thread] Putting message to be sent to calculator thread")
                        # print(messagedata.message)
                        self.robot_actors[topic].put((topic, messagedata))
                        # self.result_q.put((topic, messagedata))
                else:
                    # print(f"Got message from unknown source: {topic} -> {messagedata}")
                    continue
            except:
                continue

    def join(self, timeout=None):
        """

        :param timeout:
        """
        self.stoprequest.set()
        super(WorkerThread, self).join(timeout)


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
                    # if isinstance(message.message, AnimationUpdateMessage): #TODO: Add this line back in
                    base = message.message.robot_base
                    trajectory = message.message.trajectory
                    path = message.message.path
                    block_on_ee = message.message.block_on_ee
                    try:
                        debug_text = message.message.debug_text
                    except AttributeError:
                        debug_text = ""
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
                    # print(f"Path: {path}")
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
        self.colors = vtk_named_colors(["Firebrick", "Gray", "Firebrick", "Firebrick", "Gray", "Firebrick"])
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
            colors=self.colors, robot_id=robot, points=POINTS, block_on_end_effector=False
        )
        for link in robot_actor:
            self.pipeline.add_actor(link)
            # self.pipeline.ren.AddActor(link)
            logger.info("Added link to pipeline")
        if rendered_id:
            self.pipeline.add_actor(rendered_id)
            self.robot_texts[robot] = rendered_id

        logger.info("Should be seeing new robot, as it was just added")

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
        elapsed_time = time.time() - start_time

        text_actor.SetInput(
            f"Simulation Time: {time.strftime('%H:%M:%S', time.gmtime(elapsed_time))}\nNumber of "
            f"Robots: {len(self.robot_actors)}\nNumber of Blocks Placed: {len(self.blocks)}"
        )

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
                                new_block_tool, _, _ = add_block(
                                    (0, 0, 0),
                                    block_file_location=move_block_file_location,
                                )
                                logger.info(len(actors))
                                actors.append(new_block_tool)
                                new_block_tool.SetUserMatrix(transforms[index])
                                new_block_tool.SetScale(0.013)
                                new_block_tool.SetVisibility(True)
                                # print("Updating block end position")
                                self.pipeline.ren.AddActor(new_block_tool)
                                self.blocks[block_on_ee] = (
                                    transforms[index],
                                    new_block_tool,
                                    True,
                                )

                                # logger.info(
                                #     f"Moving block {block_on_ee} to new location {transforms[index]}"
                                # )
                            else:

                                _, new_block_tool, _ = self.blocks[block_on_ee]
                                # logger.info(f"Already know about block {block_on_ee}")
                                new_block_tool.SetUserMatrix(transforms[index])
                                color = vtk_named_colors(["Purple"])
                                new_block_tool.SetVisibility(True)
                                new_block_tool.GetProperty().SetColor(color[0])
                                # new_block_tool.SetScale(0.013)
                                self.blocks[block_on_ee] = (
                                    transforms[index],
                                    new_block_tool,
                                    True,
                                )

                                if block_on_ee in self.blocks_at_starting_location:
                                    self.blocks_at_starting_location.remove(block_on_ee)
                                    self.removed_starting_block = True
                                # logger.info(
                                #     f"Moving block {block_on_ee} to new location"
                                # )

                        elif index <= 5:
                            if index == 0:
                                if self.robot_texts[robot_id]:
                                    rendered_id = self.robot_texts[robot_id]
                                global DEBUG_TOGGLED
                                # if DEBUG_TOGGLED:
                                if DEBUG:
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
                                DEBUG_TOGGLED = False

                                rendered_id.SetUserMatrix(text_position)
                            actors[index].SetUserMatrix(transforms[index])
                            actors[index].SetScale(0.013)
                            # logger.info("Moving robot")

                    """
                    POINT
                    """
                    if POINTS:
                        assembly = actors[index]
                        assembly.SetUserMatrix(transforms[index])

                if path:
                    logger.debug(f"Path in Callback: {path}")
                    logger.debug(f"Previous path: {self.previous_path}")
                    for i in self.previous_path:
                        self.pipeline.remove_actor(i)
                        self.previous_path.remove(i)

                    for point in path:
                        self.previous_path.append(
                            self.pipeline.add_actor(cubeForPath(point))
                        )
                    self.pipeline.animate()

        while not self.block_q.empty():
            topic, message = self.block_q.get()
            if isinstance(message.message, BlockLocationMessage):
                logger.exception("GOT BLOCK, DOING SOMETHING")
                if topic in self.blocks:
                    logger.exception("BLOCK IS ALREADY KNOWN")
                    location, actor, showing = self.blocks[message.message.id]
                    actor.SetUserMatrix(
                        create_homogeneous_transform_from_point(message.message.location)
                    )
                    actor.SetVisibility(True)
                    # actor.SetPosition(message.message.location)
                    self.blocks[message.message.id] = (message.message.location, actor)
                else:
                    location = np.array(message.message.location)
                    # print(message.message.location)
                    # print(message.message.location[0])
                    # if location[0] == 0.5 and location[1] == 0.5:
                    # self.blocks_at_starting_location.append(message.message.id)
                    location[0] = float(location[0] + 0)
                    location[1] = float(location[1] + 0)
                    location[2] = float(location[2] + 0)
                    transform = np2vtk(create_homogeneous_transform_from_point(location))

                    actor, _, _ = add_block(
                        (0, 0, 0),
                        block_file_location=move_block_file_location,
                        first_time=True,
                    )

                    actor.SetUserMatrix(transform)
                    actor.SetScale(0.013)
                    actor.SetVisibility(True)
                    self.blocks[message.message.id] = (transform, actor, True)
                    self.pipeline.add_actor(actor)
                    self.pipeline.animate()
                    logger.exception("SHOWING BLOCK")

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


def Keypress(obj, event):
    key = obj.GetKeySym()
    if key == "d":
        global DEBUG
        global DEBUG_TOGGLED
        DEBUG = True if DEBUG is False else False
        DEBUG_TOGGLED = True


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
        self.LastPickedActor = None
        self.LastPickedProperty = vtk.vtkProperty()

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

    def leftButtonPressEvent(self, obj, event):
        clickPos = self.pipeline.iren.GetEventPosition()

        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.pipeline.ren)

        # get the new
        self.NewPickedActor = picker.GetActor()

        # If something was selected
        if self.NewPickedActor:
            # If we picked something before, reset its property
            if self.LastPickedActor:
                self.LastPickedActor.GetProperty().DeepCopy(self.LastPickedProperty)
            if self.NewPickedActor == self.LastPickedActor:
                self.NewPickedActor.GetProperty().DeepCopy(self.LastPickedProperty)
                return
            # Save the property of the picked actor so that we can
            # restore it next time
            self.LastPickedProperty.DeepCopy(self.NewPickedActor.GetProperty())
            # Highlight the picked actor by changing its properties
            # self.NewPickedActor.
            self.NewPickedActor.GetProperty().SetColor(1.0, 0.0, 0.0)
            self.NewPickedActor.GetProperty().SetDiffuse(1.0)
            self.NewPickedActor.GetProperty().SetSpecular(0.0)

            # save the last picked actor
            self.LastPickedActor = self.NewPickedActor

        # self.pipeline.iren.OnLeftButtonDown()
        return

    def simulate(self):
        """

        """
        # robot_actors = {}

        self.pipeline = VtkPipeline(gif_file=None)
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
        self.pipeline.iren.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)
        self.pipeline.iren.AddObserver("KeyPressEvent", Keypress)

        self.pipeline.iren.CreateRepeatingTimer(10)

        setup_structure_display(
            blueprint=BLUEPRINT,
            pipeline=self.pipeline,
            color=COLORS,
            block_file_location=block_file_location,
        )

        text_widget = vtk.vtkTextWidget()
        text_widget.SetRepresentation(text_representation)
        text_widget.SetInteractor(self.pipeline.iren)
        text_widget.SetTextActor(text_actor)
        text_widget.SelectableOff()
        text_widget.On()

        xyzLabels = ["X", "Y", "Z"]
        scale = [1.0, 1.0, 1.0]
        axes = MakeAxesActor(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)

        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(self.pipeline.iren)
        om2.EnabledOn()
        om2.InteractiveOn()

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


def axesUniversal():
    """

    :return:
    """
    axes_uni = vtk.vtkAxesActor()
    axes_uni.SetXAxisLabelText("x'")
    axes_uni.SetYAxisLabelText("y'")
    axes_uni.SetZAxisLabelText("z'")
    axes_uni.SetTipTypeToSphere()
    axes_uni.SetShaftTypeToCylinder()
    axes_uni.SetTotalLength(2, 2, 2)
    axes_uni.SetCylinderRadius(0.02)
    axes_uni.SetAxisLabels(0)

    return axes_uni


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
    sim.simulate()
    # sim.wait_for_structure_initialization(blueprint=BLUEPRINT, colors=COLORS)
    # sim.wait_for_structure_initialization()
