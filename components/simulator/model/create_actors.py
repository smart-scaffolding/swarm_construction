from random import uniform

import pkg_resources
import vtk

from .graphics import cubeForPath, vtk_named_colors


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
        actor.GetProperty().SetColor(
            uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
        )

        text = vtk.vtkVectorText()
        text.SetText(robot_id)
        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())
        text_actor = vtk.vtkActor()
        text_actor.SetMapper(text_mapper)
        text_actor.GetProperty().SetColor(
            uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
        )
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
        loc = pkg_resources.resource_filename(
            "components", "/".join(("simulator", "media", stl_files[i]))
        )
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
        loc = pkg_resources.resource_filename(
            "components", "/".join(("simulator", "media", "robot_block.stl"))
        )
        # print(loc)
        reader_list[-1].SetFileName(loc)
        mapper_list.append(vtk.vtkPolyDataMapper())
        mapper_list[-1].SetInputConnection(reader_list[i].GetOutputPort())
        actor_list.append(vtk.vtkActor())
        actor_list[-1].SetMapper(mapper_list[-1])
        actor_list[-1].GetProperty().SetColor(
            uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
        )  # (R,G,B)
        actor_list[-1].SetScale(0.013)

    text = vtk.vtkVectorText()
    text.SetText(robot_id)
    text_mapper = vtk.vtkPolyDataMapper()
    text_mapper.SetInputConnection(text.GetOutputPort())
    text_actor = vtk.vtkActor()
    text_actor.SetMapper(text_mapper)
    text_actor.GetProperty().SetColor(
        uniform(0.0, 1.0), uniform(0.0, 1.0), uniform(0.0, 1.0)
    )
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
                if blueprint[i][j][k]:
                    actor_list = vtk.vtkActor()
                    actor_list.SetMapper(block_file_location)
                    my_color = color[i][j][k]
                    actor_list.GetProperty().SetColor(my_color[0])  # (R,G,B)
                    actor_list.SetScale(0.013)
                    actor_list.SetPosition((i, j, k))
                    pipeline.add_actor(actor_list)
    return None, None, actor_list


def add_block(position=None, block_file_location=None, first_time=False):
    actor_list = vtk.vtkActor()
    actor_list.SetMapper(block_file_location)
    if first_time:
        color = vtk_named_colors(["Orange"])
    else:
        color = vtk_named_colors(["Purple"])

    actor_list.GetProperty().SetColor(color[0])  # (R,G,B)
    actor_list.SetScale(0.013)
    if position:
        actor_list.SetPosition(position)

    return actor_list, None, None


def display_path(path=None):
    actors = []
    for point in path:
        prop_assembly = cubeForPath(point)
        actors.append(prop_assembly)
    return actors


# @staticmethod
def setup_file_names(num):
    file_names = []
    for i in range(0, num):
        file_names.append("link" + str(i) + ".stl")

    return file_names
