import math

import vtk


class VtkPipeline:
    """
    This class is used to render components to the VTK renderer pipeline. All rendered components should interact
    with the rendering pipeline. 

    """

    def __init__(
        self,
        background=(0.15, 0.15, 0.15),
        total_time_steps=None,
        timer_rate=60,
        gif_file=None,
    ):
        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(background[0], background[1], background[2])
        self.ren_win = vtk.vtkRenderWindow()
        self.ren_win.AddRenderer(self.ren)
        self.ren_win.SetSize((3000, 3000))
        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.ren_win)
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

        self.actor_list = []
        self.mapper_list = []
        self.source_list = []
        self.screenshot_count = 0
        self.timer_rate = timer_rate
        self.gif_data = []

        if gif_file is not None:
            try:
                assert type(gif_file) is str
            except AssertionError:
                gif_file = str(gif_file)
            self.gif_file = gif_file
        else:
            self.gif_file = None

        if total_time_steps is not None:
            assert type(total_time_steps) is int
            self.timer_count = 0
            self.total_time_steps = total_time_steps

    def render(self, ui=True):
        for each in self.actor_list:
            self.ren.AddActor(each)
        self.ren.ResetCamera()
        self.set_camera()
        self.ren_win.Render()
        self.ren_win.SetWindowName("Robot Simulation")
        if ui:
            self.iren.Initialize()
            self.iren.Start()

    def add_actor(self, actor):
        self.actor_list.append(actor)
        return actor

    def remove_actor(self, actor):
        try:
            self.actor_list.remove(actor)
            self.ren.RemoveActor(actor)
            return True
        except ValueError as e:
            print(e)
            return False

    def set_camera(self):
        cam = self.ren.GetActiveCamera()
        self.ren.SetActiveCamera(cam)

    def animate(self):
        self.ren.ResetCamera()
        self.ren_win.Render()
        self.iren.Initialize()
        self.iren.CreateRepeatingTimer(
            math.floor(1000 / self.timer_rate)
        )  # Timer creates 60 fps
        self.render()

    def screenshot(self, filename=None):
        w2if = vtk.vtkWindowToImageFilter()
        w2if.SetInput(self.ren_win)
        w2if.Update()
        if filename is None:
            filename = "screenshot"
        filename = filename + "%d.png" % self.screenshot_count
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(filename)
        self.screenshot_count += 1
        writer.SetInputData(w2if.GetOutput())
        writer.Write()

    def timer_tick(self):
        import imageio

        self.timer_count += 1

        if self.timer_count >= self.total_time_steps:
            self.iren.DestroyTimer()
            if self.gif_file is not None:
                assert len(self.gif_data) > 0
                imageio.mimsave(self.gif_file + ".gif", self.gif_data)
                import os

                for i in range(self.screenshot_count):
                    os.remove(self.gif_file + "%d.png" % i)
                return

        if self.gif_file is not None:
            if (self.timer_count % 60) == 0:
                self.screenshot(self.gif_file)
                path = self.gif_file + "%d.png" % (self.screenshot_count - 1)
                self.gif_data.append(imageio.imread(path))

    @staticmethod
    def vtk_named_colors(colors):
        """
        Returns a list of vtk colors
        :param colors: List of color names supported by vtk
        :return: A list of vtk colors
        """
        if type(colors) is not list:
            colors = [colors]
        colors_rgb = [0] * len(colors)
        for i in range(len(colors)):
            colors_rgb[i] = list(vtk.vtkNamedColors().GetColor3d(colors[i]))
        return colors_rgb


def cube_for_path(point):
    """
    Creates a cube that can be used to show the path a robot is expected to take.

    NOTE: This method is currently unused.

    """

    colors = vtk.vtkNamedColors()
    direction = point[3]
    cube_source = vtk.vtkCubeSource()
    cube_source.SetCenter((point[0] + 0.49, point[1] + 0.49, point[2] + 0.49))

    cube_source.Update()
    face_colors = vtk.vtkUnsignedCharArray()
    face_colors.SetNumberOfComponents(3)

    face_x_plus = colors.GetColor3ub("DarkGreen")
    face_x_minus = colors.GetColor3ub("DarkGreen")
    face_y_plus = colors.GetColor3ub("DarkGreen")
    face_y_minus = colors.GetColor3ub("DarkGreen")
    face_z_plus = colors.GetColor3ub("DarkGreen")
    face_z_minus = colors.GetColor3ub("DarkGreen")

    if direction == "top":
        cube_source.SetXLength(0.25)
        cube_source.SetYLength(0.25)
        cube_source.SetZLength(1.1)  # 1.1
        face_z_plus = colors.GetColor3ub("Cyan")

    if direction == "bottom":
        face_z_minus = colors.GetColor3ub("Cyan")

    if direction == "right":
        face_x_plus = colors.GetColor3ub("Cyan")

    if direction == "left":
        cube_source.SetXLength(1)
        cube_source.SetYLength(0.25)
        cube_source.SetZLength(0.25)
        face_x_minus = colors.GetColor3ub("Cyan")

    if direction == "front":
        face_y_minus = colors.GetColor3ub("Cyan")

    if direction == "back":
        face_y_plus = colors.GetColor3ub("Cyan")

    face_colors.InsertNextTypedTuple(face_x_minus)
    face_colors.InsertNextTypedTuple(face_x_plus)
    face_colors.InsertNextTypedTuple(face_y_minus)
    face_colors.InsertNextTypedTuple(face_y_plus)
    face_colors.InsertNextTypedTuple(face_z_minus)
    face_colors.InsertNextTypedTuple(face_z_plus)

    cube_source.GetOutput().GetCellData().SetScalars(face_colors)
    cube_source.Update()

    cube_mapper = vtk.vtkPolyDataMapper()
    cube_mapper.SetInputData(cube_source.GetOutput())
    cube_mapper.Update()

    cube_actor = vtk.vtkActor()
    cube_actor.SetMapper(cube_mapper)
    return cube_actor


def circle_for_trajectory(point, direction, index=None):
    """
    Creates a circle that can be used for showing trajectories.

    NOTE: This method is currently unused.

    """
    colors = vtk.vtkNamedColors()

    source = vtk.vtkSphereSource()
    if direction == "left":
        point[0] = point[0] + 1.37
        point[2] = point[2] + 1.37
    source.SetCenter(point)
    source.SetRadius(0.09)

    source.Update()

    circle_mapper = vtk.vtkPolyDataMapper()
    circle_mapper.SetInputData(source.GetOutput())
    circle_mapper.Update()

    circle_actor = vtk.vtkActor()
    circle_actor.SetMapper(circle_mapper)
    circle_actor.GetProperty().SetColor(colors.GetColor3ub("Red"))  # Color red
    if index == 0:
        circle_actor.GetProperty().SetColor(colors.GetColor3ub("Blue"))

    prop_assembly = vtk.vtkPropAssembly()
    prop_assembly.AddPart(circle_actor)
    return prop_assembly
