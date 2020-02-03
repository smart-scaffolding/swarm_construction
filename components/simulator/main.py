from __future__ import print_function
import vtk
import numpy as np
import zmq
import sys
import time
import threading
from queue import Queue
from random import choice
from components.simulator.model.model import Inchworm
from components.simulator.model.graphics import *
from components.simulator.model.create_actors import *
# from components.simulator.model.graphics import *
from components.simulator.common.common import create_homogeneous_transform_from_point
from components.simulator.common.transforms import np2vtk
import zlib
import pickle


POINTS = True
ROBOTS = 1
# BLUEPRINT = np.array([
#         [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
#         [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
#         [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
#         [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
#         [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
#         [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
#     ])

BLUEPRINT = np.array([
                             [[1] * 1] * 9,
                         ] * 9)
bx, by, bz = BLUEPRINT.shape
# COLORS = [[["DarkGreen"] * bz] * by] * bx
COLORS = [[[vtk_named_colors(["DarkGreen"])] * bz] * by] * bx

class WorkerThread(threading.Thread):
    def __init__(self, dir_q, result_q, filter_q, socket, pipeline):
        super(WorkerThread, self).__init__()
        self.robot_actors = {}
        self.dir_q = dir_q
        self.result_q = result_q
        self.filter_q = filter_q
        self.stoprequest = threading.Event()
        self.socket = socket
        self.pipeline = pipeline


    def run(self):
        while not self.stoprequest.isSet():
            try:
                # dirname = self.dir_q.get(True, 0.05)
                [topic, message] = self.socket.recv_multipart()
                message = zlib.decompress(message)
                messagedata = pickle.loads(message)
                print(f"[Worker thread]: {topic} {messagedata}")
                if topic not in self.robot_actors:
                    print("[Worker thread]: Received new robot connection, adding to queue")
                    self.robot_actors[topic] = Queue()
                    self.robot_actors[topic].put((topic, messagedata))
                    self.filter_q.put((topic, messagedata, self.robot_actors[topic])) # add new robot
                    #create new worker here

                    # self.result_q.put((topic, messagedata))
                else:
                    # print(f"[Worker thread] Putting message to be sent to calculator thread")
                    self.robot_actors[topic].put((topic, messagedata))
                    # self.result_q.put((topic, messagedata))
            except:
                continue

    def join(self, timeout=None):
        self.stoprequest.set()
        super(WorkerThread, self).join(timeout)


class CalculatorThread(WorkerThread):
    def __init__(self, dir_q, result_q, filter_q, socket, pipeline):
        super(CalculatorThread, self).__init__(dir_q, result_q, filter_q, socket, pipeline)


    def run(self):
        while not self.stoprequest.isSet():
            # print("[Calculator thread] Alive")
            # try:
            if not self.dir_q.empty():
                # print("[Calculator thread] Getting robot to calculate")
                actor, message = self.dir_q.get()

                """
                DISPLAY ROBOTS
                """
                if not POINTS:
                    base = np.matrix([[1, 0, 0, 0.5],
                                     [0, 1, 0, 0.5],
                                     [0, 0, 1, 1.],
                                     [0, 0, 0, 1]])
                    base1 = np.matrix([[1, 0, 0, 1.5],
                                      [0, 1, 0, 0.5],
                                      [0, 0, 1, 1.],
                                      [0, 0, 0, 1]])
                    base2 = np.matrix([[1, 0, 0, 1.5],
                                      [0, 1, 0, 1.5],
                                      [0, 0, 1, 1.],
                                      [0, 0, 0, 1]])
                    base3 = np.matrix([[1, 0, 0, 4.5],
                                      [0, 1, 0, 0.5],
                                      [0, 0, 1, 1.],
                                      [0, 0, 0, 1]])
                    base4 = np.matrix([[1, 0, 0, 4.5],
                                      [0, 1, 0, 1.5],
                                      [0, 0, 1, 3.],
                                      [0, 0, 0, 1]])
                    base5 = np.matrix([[1, 0, 0, 5.5],
                                      [0, 1, 0, 0.5],
                                      [0, 0, 1, 1.],
                                      [0, 0, 0, 1]])
                    if actor == b"ROBOT_1":
                        base = base
                    elif actor == b"ROBOT_2":
                        base = base5
                    else:
                        base = base4
                    # base = choice([base, base4])
                    trajectory1 = np.array([[0, 0, 0, 0]])
                    trajectory2 = np.array([[0, 0, np.pi/2, 0]])
                    trajectory3 = np.array([[0, 0, 0, np.pi/2]])
                    trajectory4 = np.array([[0, 0, np.pi/2, np.pi/2]])
                    trajectory5 = np.array([[0, np.pi/2, 0, 0]])



                    num_points = 60
                    trajectory_choice = choice([trajectory1, trajectory2, trajectory3, trajectory4, trajectory5])
                    robot = Inchworm(base=base)

                    forward_1 = np.transpose(np.asmatrix(np.linspace(float(trajectory1[0][0]), trajectory_choice[0][0],
                                                                     num_points)))
                    forward_2 = np.transpose(np.asmatrix(np.linspace(float(trajectory1[0][1]), trajectory_choice[0][1],
                                                                     num_points)))
                    forward_3 = np.transpose(np.asmatrix(np.linspace(float(trajectory1[0][2]), trajectory_choice[0][2],
                                                                     num_points)))
                    forward_4 = np.transpose(np.asmatrix(np.linspace(float(trajectory1[0][3]), trajectory_choice[0][3],
                                                                     num_points)))

                    multiple_trajectories = np.concatenate((forward_1, forward_2, forward_3, forward_4), axis=1)

                    for trajectory in multiple_trajectories:
                        transform, robot_actors = robot.fkine(stance=trajectory, apply_stance=True)
                        self.result_q.put((actor, robot_actors))


                """
                DISPLAY POINTS
                """
                if POINTS:

                    # base1 = np.matrix([[1, 0, 0, 1.5],
                    #                    [0, 1, 0, 0.5],
                    #                    [0, 0, 1, 1.],
                    #                    [0, 0, 0, 1]])
                    # base2 = np.matrix([[1, 0, 0, 1.5],
                    #                    [0, 1, 0, 1.5],
                    #                    [0, 0, 1, 1.],
                    #                    [0, 0, 0, 1]])
                    # base3 = np.matrix([[1, 0, 0, 4.5],
                    #                    [0, 1, 0, 0.5],
                    #                    [0, 0, 1, 1.],
                    #                    [0, 0, 0, 1]])
                    # base4 = np.matrix([[1, 0, 0, 4.5],
                    #                    [0, 1, 0, 1.5],
                    #                    [0, 0, 1, 3.],
                    #                    [0, 0, 0, 1]])
                    # base5 = np.matrix([[1, 0, 0, 5.5],
                    #                    [0, 1, 0, 0.5],
                    #                    [0, 0, 1, 1.],
                    #                    [0, 0, 0, 1]])
                    #
                    new_position = message.message.robot_base
                    # homo_trans = create_homogeneous_transform_from_point(new_position)
                    # print(message.message)
                    # base = choice([base1, base2, base3, base4, base5])
                    transform = np2vtk(new_position)
                    self.result_q.put((actor, [transform]))

            # except:
            #     continue





class vtkTimerCallback():
    def __init__(self, renderer, renderWindow, queue, new_actors, dir_q, result_q, socket, pipeline):
        self.timer_count = 0
        self.robot_actors = {}
        self.renderer = renderer
        self.renderWindow = renderWindow
        self.queue = queue
        self.new_actors = new_actors
        self.worker_pool = []
        # self.robot_update = robot_update
        self.dir_q = dir_q
        self.result_q = result_q
        self.socket = socket
        self.pipeline = pipeline
        self.colors = vtk_named_colors(["Red", "Blue", "Blue", "Purple"])


    def add_robot_to_sim(self, robot, result_queue):
        base = np.matrix([[1, 0, 0, 0.5],
                          [0, 1, 0, 0.5],
                          [0, 0, 1, 1.],
                          [0, 0, 0, 1]])
        new_robot = Inchworm(base=base, blueprint=BLUEPRINT)

        _, robot_actor, _ = setup_pipeline_objs(colors=self.colors, points=True)
        for link in robot_actor:
            self.pipeline.add_actor(link)
        self.robot_actors[robot] = (robot_actor, new_robot, result_queue)
        # print(f"Robot added to known robots: {self.robot_actors}")
        self.pipeline.animate()

        # sphereSource = vtk.vtkSphereSource()
        # sphereSource.SetCenter(-10.0, -10.0, -10.0)
        # sphereSource.SetRadius(5)
        #
        # # Create a mapper and actor
        # mapper = vtk.vtkPolyDataMapper()
        # mapper.SetInputConnection(sphereSource.GetOutputPort())
        # actor = vtk.vtkActor()
        # actor.SetMapper(mapper)
        # actor.GetProperty().SetColor(uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0))
        # prop = actor.GetProperty()
        # self.renderer.AddActor(actor)
        # self.robot_actors[robot] = actor
        # self.create_new_thread(robot)
        # self.renderer.GetRenderWindow().Render()
        # self.renderWindow.Render()
        # print("Callback: Should now be seeing robot")

    def create_new_thread(self, queue, result_queue):
        print("Callback: Creating new thread for robot")
        calculate_thread = CalculatorThread(dir_q=queue, result_q=result_queue, filter_q=self.new_actors,
                                            socket=self.socket, pipeline=self.pipeline)
        self.worker_pool.append(calculate_thread)
        calculate_thread.start()

    def execute(self,obj,event):
        # string = self.socket.recv()
        # topic, messagedata = string.split()
        # print(topic, messagedata)
        if self.timer_count % 10 == 0:
           print(self.timer_count)
        self.timer_count += 1
        while not self.new_actors.empty():
           topic, message, queue = self.new_actors.get()
           result_q = Queue()
           print(f"Callback: Creating new thread for actor: {topic} {queue}")
           self.create_new_thread(queue, result_q)
           print(f"Callback: Adding new actor to sim: {topic}")
           self.add_robot_to_sim(topic, result_q)

        for robot in self.robot_actors:
            actors, model, robot_queue = self.robot_actors[robot]
            if not robot_queue.empty():
        # if not self.queue.empty():
        #     for i in range(5):
               robot_id, transforms = robot_queue.get()
               # print(f"Callback: Moving position of robot: {robot_id}")
               # print(f"Known actors in callback: {self.robot_actors}")

               for index in range(len(transforms)):

                   """
                   ROBOT
                   """
                   if not POINTS:
                       actors[index].SetUserMatrix(transforms[index])
                       actors[index].SetScale(0.013)

                   """
                   POINT
                   """
                   if POINTS:
                       actors[index].SetUserMatrix(transforms[index])

               # x, y, z = edit_actor.GetPosition()
               # self.robot_actors[actor].SetPosition(x + int(message), y + int(message), 0)
        iren = obj
        iren.GetRenderWindow().Render()


# class vtkCallback(vtkCallbackCommand):
#     def __init__(self):
#         pass
#
#     def Execute(self, vtkObject, p_int, void):
#         pass


class Simulate:
    def __init__(self, robot_update, dir_q, result_q, new_actors, socket):
        self.robot_actors = {}
        self.worker_pool = []
        self.robot_update = robot_update
        self.dir_q = dir_q
        self.result_q = result_q
        self.new_actors = new_actors
        self.socket = socket

    # def add_robot_to_sim(self, robot):
    #     print(f"Simulator: Adding new robot to sim: {robot}")
    #     sphereSource = vtk.vtkSphereSource()
    #     sphereSource.SetCenter(-10.0, -10.0, -10.0)
    #     sphereSource.SetRadius(5)
    #
    #     # Create a mapper and actor
    #     mapper = vtk.vtkPolyDataMapper()
    #     mapper.SetInputConnection(sphereSource.GetOutputPort())
    #     actor = vtk.vtkActor()
    #     actor.SetMapper(mapper)
    #     prop = actor.GetProperty()
    #     self.renderer.AddActor(actor)
    #     self.robot_actors[robot] = actor
    #     self.create_new_thread(robot)
    #     self.renderWindow.Render()


    def wait_for_structure_initialization(self, blueprint=None, colors=None):
        global BLUEPRINT
        global COLORS
        if blueprint is not None:
            BLUEPRINT = blueprint
        if colors is not None:
            COLORS = colors
        else:
            # while True:
                # try:
            [topic, message] = self.socket.recv_multipart()
            message = zlib.decompress(message)
            messagedata = pickle.loads(message)
            print(f"[Worker thread]: {topic} {messagedata}")
            if topic == b"STRUCTURE":
                BLUEPRINT = messagedata.message.blueprint
                COLORS = messagedata.message.colors
                print(BLUEPRINT)
                print(COLORS)
                    # break
                # except:
            #     continue
        self.simulate()

    def simulate(self):
        # robot_actors = {}

        self.pipeline = VtkPipeline(gif_file=None)
        # self.pipeline.reader_list, self.pipeline.actor_list, self.pipeline.mapper_list = setup_pipeline_objs()
        self.param = {
            "cube_axes_x_bounds": np.array([[0, len(BLUEPRINT)]]),
            "cube_axes_y_bounds": np.array([[0, len(BLUEPRINT[0])]]),
            "cube_axes_z_bounds": np.array([[0, len(BLUEPRINT[0][0])]]),
            "floor_position": np.array([[0, 0, 0]])
        }


        cube_axes = axesCubeFloor(self.pipeline.ren,
                                  self.param.get("cube_axes_x_bounds"),
                                  self.param.get("cube_axes_y_bounds"),
                                  self.param.get("cube_axes_z_bounds"),
                                  self.param.get("floor_position"))

        self.pipeline.add_actor(cube_axes)






        # self.renderer = vtk.vtkRenderer()
        # self.renderer.SetBackground(0.15, 0.15, 0.15)  # Background color white
        # self.renderWindow = vtk.vtkRenderWindow()
        # # renderWindow.SetWindowName("Test")
        # self.renderWindow.SetSize((3000, 3000))
        #
        # self.renderWindow.AddRenderer(self.renderer)
        # renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        # renderWindowInteractor.SetRenderWindow(self.renderWindow)
        # renderWindowInteractor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        #
        # self.renderer.AddActor(axesUniversal())
        #
        # # Render and interact
        # self.renderWindow.Render()
        #
        # # Initialize must be called prior to creating timer events.
        # renderWindowInteractor.Initialize()



        # Sign up to receive TimerEvent
        cb = vtkTimerCallback(new_actors=self.new_actors, renderer=self.pipeline.ren,
                              renderWindow=self.pipeline.ren_win,
                              queue=self.robot_update, dir_q=self.dir_q, result_q=self.result_q, socket=self.socket,
                              pipeline=self.pipeline)
        # cb.actors = self.robot_actors
        # cb.socket = socket
        # cb.queue = result_q

        # renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
        #
        # # for i in range(20):
        # #     renderWindowInteractor.InvokeEvent(custom_1)
        #
        # timerId = renderWindowInteractor.CreateRepeatingTimer(math.floor(1000 / 60))

        # Create the "thread pool"

        self.pipeline.iren.AddObserver('TimerEvent', cb.execute)

        start = time.time()
        print(start)

        # print(f"DISPLAYING STRUCTURE {BLUEPRINT} {COLORS}")
        setup_structure_display(blueprint=BLUEPRINT, pipeline=self.pipeline, color=COLORS)

        print(time.time()-start)

        # self.pipeline.add_actor(structure_actor)
        # if display_path:
        #     self.pipeline.add_actor(self._display_path())

        xyzLabels = ['X', 'Y', 'Z']
        scale = [1.0, 1.0, 1.0]
        axes = MakeAxesActor(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)
        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(self.pipeline.iren)
        om2.EnabledOn()
        om2.InteractiveOn()

        # self.forward_kinematics_workers = [cb.create_new_thread() for i in range(4)]

        # for thread in self.forward_kinematics_workers:
        #     thread.start()

        pool = [WorkerThread(dir_q=self.dir_q, result_q=self.result_q, filter_q=self.new_actors, socket=socket,
                             pipeline=self.pipeline)]

        # Start all threads
        for thread in pool:
            thread.start()
            print("Started worker thread")

        try:
            # renderWindowInteractor.Start()
            self.pipeline.animate()
        except KeyboardInterrupt:
            pass
        finally:
            for thread in pool:
                thread.join()
            for thread in self.worker_pool:
                thread.join()
            # for thread in self.forward_kinematics_workers:
            #     thread.join()

def axesUniversal():
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



if __name__ == '__main__':
    port = "5559"
    if len(sys.argv) > 1:
        port = sys.argv[1]
        int(port)

    if len(sys.argv) > 2:
        port1 = sys.argv[2]
        int(port1)

    # Socket to talk to server
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # socket1 = context.socket(zmq.SUB)
    print("Collecting updates from simulator...")
    socket.bind("tcp://127.0.0.1:5559")
    # socket1.connect(f"tcp://localhost:{port}")

    if len(sys.argv) > 2:
        socket.connect(f"tcp://localhost:{port1}")
        # socket1.connect(f"tcp://localhost:{port1}")

    # Subscribe to zipcode, default is NYC, 10001
    # topicfilter = [b"10001", b"10002", b"10003", b"10004", b"10005", b"10006"]
    # for topic in topicfilter:
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    # socket1.setsockopt_string(zmq.SUBSCRIBE, "10002")

    # Process 5 updates
    total_value = 0
    # for update_nbr in range(5):
    dir_q = Queue()
    result_q = Queue()
    new_actors = Queue()
    robot_update = Queue()

    sim = Simulate(robot_update=robot_update, dir_q=dir_q, result_q=result_q, new_actors=new_actors, socket=socket)
    sim.simulate()
    # sim.wait_for_structure_initialization(blueprint=BLUEPRINT, colors=COLORS)
    # sim.wait_for_structure_initialization()
