import vtk
from .graphics import VtkPipeline
from .graphics import axesCube
from .graphics import axesCubeFloor
from .graphics import vtk_named_colors
from .graphics import cubeForPath, circleForTrajectory
from .graphics import MakeAxesActor
from .graphics import vtk_named_colors, vtk_named_colors_3d
import pkg_resources
from random import uniform


def setup_pipeline_objs(colors, robot_id, points=False, block_on_end_effector=False):
    """
    Internal function to initialise vtk objects.
    :return: reader_list, actor_list, mapper_list
    """

    if points:
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetCenter(0, 0, 0)
        sphereSource.SetRadius(0.3)

        # Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0))

        text = vtk.vtkVectorText()
        text.SetText(robot_id)
        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())
        text_actor = vtk.vtkActor()
        text_actor.SetMapper(text_mapper)
        text_actor.GetProperty().SetColor(uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0))
        text_actor.AddPosition(0, 0, 1)
        text_actor.RotateX(60)
        text_actor.SetScale(0.5)

        assembly = vtk.vtkAssembly()
        assembly.AddPart(actor)
        assembly.AddPart(text_actor)


        return [assembly], []
    stl_files = setup_file_names(4)
    print("STL FILES: {}".format(stl_files))
    reader_list = [0] * len(stl_files)
    actor_list = [0] * len(stl_files)
    # print("Actor List: {}".format(actor_list))

    mapper_list = [0] * len(stl_files)
    for i in range(len(stl_files)):
        reader_list[i] = vtk.vtkSTLReader()
        loc = pkg_resources.resource_filename("components", '/'.join(('simulator','media', stl_files[i])))
        # print(loc)
        reader_list[i].SetFileName(loc)
        mapper_list[i] = vtk.vtkPolyDataMapper()
        mapper_list[i].SetInputConnection(reader_list[i].GetOutputPort())
        actor_list[i] = vtk.vtkActor()
        actor_list[i].SetMapper(mapper_list[i])
        actor_list[i].GetProperty().SetColor(colors[i])  # (R,G,B)
        actor_list[i].SetScale(0.013)

    if block_on_end_effector:
        reader_list.append(vtk.vtkSTLReader())
        loc = pkg_resources.resource_filename("components", '/'.join(('simulator','media', "robot_block.stl")))
        # print(loc)
        reader_list[-1].SetFileName(loc)
        mapper_list.append(vtk.vtkPolyDataMapper())
        mapper_list[-1].SetInputConnection(reader_list[i].GetOutputPort())
        actor_list.append(vtk.vtkActor())
        actor_list[-1].SetMapper(mapper_list[-1])
        actor_list[-1].GetProperty().SetColor(uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0))  # (R,G,B)
        actor_list[-1].SetScale(0.013)

    text = vtk.vtkVectorText()
    text.SetText(robot_id)
    text_mapper = vtk.vtkPolyDataMapper()
    text_mapper.SetInputConnection(text.GetOutputPort())
    text_actor = vtk.vtkActor()
    text_actor.SetMapper(text_mapper)
    text_actor.GetProperty().SetColor(uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0))
    text_actor.AddPosition(0, 0, 1)
    text_actor.RotateX(60)
    text_actor.SetScale(0.5)

    # assembly = vtk.vtkAssembly()
    # for actor in actor_list:
    #     assembly.AddPart(actor)
    # assembly.AddPart(text_actor)

    return actor_list, text_actor


def setup_structure_display(blueprint, pipeline, color, block_file_location):
    """
    Internal function to initialise vtk objects.
    :return: reader_list, actor_list, mapper_list
    """
    # reader_list = np.zeros(self.blueprint.size)
    # actor_list = np.zeros(self.blueprint.size)
    # print("Actor List: {}".format(actor_list))
    #
    # mapper_list = np.zeros(self.blueprint.size)
    # for i in range(len(self.stl_files)):

    for i in range(len(blueprint)):
        for j in range(len(blueprint[0])):
            for k in range(len(blueprint[0][0])):
                if (blueprint[i][j][k]):
                    # reader_list = vtk.vtkSTLReader()
                    # # loc = pkg_resources.resource_filename("components", '/'.join(('simulator','media', "block.stl")))
                    # loc = block_file_location
                    # # print(loc)
                    # reader_list.SetFileName(loc)
                    # mapper_list = vtk.vtkPolyDataMapper()
                    # mapper_list.SetInputConnection(reader_list.GetOutputPort())
                    actor_list = vtk.vtkActor()
                    actor_list.SetMapper(block_file_location)
                    # color_index = random.randint(1, 3)
                    # if i == 0 or j == 0 or k == 0 or i ==(len(self.blueprint-1)) or j ==(len(self.blueprint[
                    #                                                                              0]-1)) or k ==(
                    #         len(self.blueprint[0][0]-1)):
                    #     color_index = 1
                    # if color_index == 1:
                    #     color = vtk_named_colors(["DarkGreen"])
                    # elif color_index == 2:
                    #     color = vtk_named_colors(["Red"])
                    # else:
                    #     color = vtk_named_colors(["LightGreen"])
                    # vtk_color = vtk_named_colors(["DarkGreen"])
                    my_color = color[i][j][k]
                    # print(my_color)
                    # vtk_color = vtk_named_colors(my_color)
                    actor_list.GetProperty().SetColor(my_color[0])  # (R,G,B)
                    # actor_list.GetProperty().SetColor(color[i][j][k])
                    actor_list.SetScale(0.013)
                    actor_list.SetPosition((i, j, k))
                    # print("SCALE: {}".format(actor_list.GetScale()))
                    # print("POSITION: {}".format(actor_list.GetPosition()))
                    pipeline.add_actor(actor_list)
    return None, None, actor_list


def add_block(position=None, block_file_location=None):
    # reader_list = vtk.vtkSTLReader()
    # # loc = pkg_resources.resource_filename("components", '/'.join(('simulator','media', "block.stl")))
    # loc = block_file_location
    # # print(loc)
    # reader_list.SetFileName(loc)
    # mapper_list = vtk.vtkPolyDataMapper()
    # mapper_list.SetInputConnection(reader_list.GetOutputPort())
    actor_list = vtk.vtkActor()
    actor_list.SetMapper(block_file_location)
    color = vtk_named_colors(["Purple"])

    actor_list.GetProperty().SetColor(color[0])  # (R,G,B)
    actor_list.SetScale(0.013)
    if position:
        actor_list.SetPosition(position)
    # print("Adding block at pos: {}".format(position))
    # self.pipeline.add_actor(actor_list)

    return actor_list, None, None


def display_path(path=[(0, 0, 0, "top"), (1, 0, 0, "top"), (2, 0, 0, "top"), (3, 0, 0, "top"), (4, 0, 0, "top"),
                              (5, 0, 0, "top"), ]):
    actors = []
    for point in path:
        prop_assembly = cubeForPath(point)
        actors.append(prop_assembly)
        # self.pipeline.add_actor(prop_assembly)
    return actors


# @staticmethod
def setup_file_names(num):
    file_names = []
    for i in range(0, num):
        file_names.append('link' + str(i) + '.stl')

    return file_names