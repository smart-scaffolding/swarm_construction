from __future__ import print_function
import vtk
from time import sleep
from vtk import vtkCallbackCommand
from multiprocessing import Process

import threading
import zmq
import sys
import math
import os, time
import threading
from queue import Queue
from random import uniform, randint

ROBOTS = 1

class WorkerThread(threading.Thread):
    def __init__(self, dir_q, result_q, filter_q, socket):
        super(WorkerThread, self).__init__()
        self.robot_actors = {}
        self.dir_q = dir_q
        self.result_q = result_q
        self.filter_q = filter_q
        self.stoprequest = threading.Event()
        self.socket = socket


    def run(self):
        while not self.stoprequest.isSet():
            try:
                # dirname = self.dir_q.get(True, 0.05)
                string = self.socket.recv()
                topic, messagedata = string.split()
                print(f"Worker thread: {topic} {messagedata}")
                if topic not in self.robot_actors:
                    print("Worker thread: Received new robot connection, adding to queue")
                    self.robot_actors[topic] = topic
                    self.filter_q.put((topic, messagedata))
                self.result_q.put((topic, messagedata))
            except:
                continue

    def join(self, timeout=None):
        self.stoprequest.set()
        super(WorkerThread, self).join(timeout)


class CalculatorThread(WorkerThread):
    def __init__(self, dir_q, result_q, filter_q, socket):
        super(CalculatorThread, self).__init__(dir_q, result_q, filter_q, socket)


    def run(self):
        while not self.stoprequest.isSet():
            try:
                if not self.dir_q.empty():
                    actor, message = self.dir_q.get()
                    print("Calculator Thread: Calculating something")
                    time.sleep(randint(1, 5))
                    self.result_q.put((actor, message))
            except:
                continue





class vtkTimerCallback():
    def __init__(self, renderer, renderWindow, queue, new_actors, dir_q, result_q, socket):
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


    def add_robot_to_sim(self, robot):
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetCenter(-10.0, -10.0, -10.0)
        sphereSource.SetRadius(5)

        # Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0))
        prop = actor.GetProperty()
        self.renderer.AddActor(actor)
        self.robot_actors[robot] = actor
        self.create_new_thread(robot)
        self.renderer.GetRenderWindow().Render()
        self.renderWindow.Render()
        print("Callback: Should now be seeing robot")

    def create_new_thread(self, robot):
        print("Callback: Creating new thread for robot")
        calculate_thread = CalculatorThread(dir_q=self.result_q, result_q=self.queue, filter_q=self.new_actors,
                                            socket=self.socket)
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
           topic, message = self.new_actors.get()
           print(f"Callback: Adding new actor to sim: {topic}")
           self.add_robot_to_sim(topic)
        if not self.queue.empty():
           actor, message = self.queue.get()
           print(f"Callback: Moving position of robot: {actor}")
           edit_actor = self.robot_actors[actor]
           x, y, z = edit_actor.GetPosition()
           self.robot_actors[actor].SetPosition(x + int(message), y + int(message), 0)
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



    def simulate(self):
        # robot_actors = {}
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.15, 0.15, 0.15)  # Background color white
        self.renderWindow = vtk.vtkRenderWindow()
        # renderWindow.SetWindowName("Test")
        self.renderWindow.SetSize((3000, 3000))

        self.renderWindow.AddRenderer(self.renderer)
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(self.renderWindow)
        renderWindowInteractor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

        self.renderer.AddActor(axesUniversal())

        # Render and interact
        self.renderWindow.Render()

        # Initialize must be called prior to creating timer events.
        renderWindowInteractor.Initialize()



        # Sign up to receive TimerEvent
        cb = vtkTimerCallback(new_actors=self.new_actors, renderer=self.renderer, renderWindow=self.renderWindow,
                              queue=self.robot_update, dir_q=self.dir_q, result_q=self.result_q, socket=self.socket)
        # cb.actors = self.robot_actors
        # cb.socket = socket
        # cb.queue = result_q

        renderWindowInteractor.AddObserver('TimerEvent', cb.execute)

        # for i in range(20):
        #     renderWindowInteractor.InvokeEvent(custom_1)

        timerId = renderWindowInteractor.CreateRepeatingTimer(math.floor(1000 / 60))

        # Create the "thread pool"
        pool = [WorkerThread(dir_q=self.dir_q, result_q=self.result_q, filter_q=self.new_actors, socket=socket) for i
                in range(ROBOTS)]

        # Start all threads
        for thread in pool:
            thread.start()

        try:
            renderWindowInteractor.Start()
        except KeyboardInterrupt:
            pass
        finally:
            for thread in pool:
                thread.join()
            for thread in self.worker_pool:
                thread.join()

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
    print("Collecting updates...")
    socket.connect(f"tcp://localhost:{port}")
    # socket1.connect(f"tcp://localhost:{port}")

    if len(sys.argv) > 2:
        socket.connect(f"tcp://localhost:{port1}")
        # socket1.connect(f"tcp://localhost:{port1}")


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