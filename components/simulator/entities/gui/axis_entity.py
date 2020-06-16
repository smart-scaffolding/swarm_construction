import numpy as np
import vtk


class AxisFactory:
    """
    Returns an XYZ axis object (for widgets, structure)
    """

    @staticmethod
    def axes_cube(
        ren,
        x_bound=np.matrix([[-1.5, 1.5]]),
        y_bound=np.matrix([[-1.5, 1.5]]),
        z_bound=np.matrix([[-1.5, 1.5]]),
    ):
        """
        Helper method to create a VTK cube axis actor

        """
        cube_axes_actor = vtk.vtkCubeAxesActor()
        cube_axes_actor.SetBounds(
            x_bound[0, 0],
            x_bound[0, 1],
            y_bound[0, 0],
            y_bound[0, 1],
            z_bound[0, 0],
            z_bound[0, 1],
        )
        cube_axes_actor.SetCamera(ren.GetActiveCamera())
        cube_axes_actor.GetTitleTextProperty(0).SetColor(1.0, 0.0, 0.0)
        cube_axes_actor.GetLabelTextProperty(0).SetColor(1.0, 0.0, 0.0)

        cube_axes_actor.GetTitleTextProperty(1).SetColor(0.0, 1.0, 0.0)
        cube_axes_actor.GetLabelTextProperty(1).SetColor(0.0, 1.0, 0.0)

        cube_axes_actor.GetTitleTextProperty(2).SetColor(0.0, 0.0, 1.0)
        cube_axes_actor.GetLabelTextProperty(2).SetColor(0.0, 0.0, 1.0)

        cube_axes_actor.SetFlyModeToStaticTriad()

        return cube_axes_actor

    @staticmethod
    def axes_cube_floor(
        ren,
        x_bound=np.matrix([[-1.5, 1.5]]),
        y_bound=np.matrix([[-1.5, 1.5]]),
        z_bound=np.matrix([[-1.5, 1.5]]),
    ):
        """
        Creates a large axis for the main window of the simulator

        """
        axes = AxisFactory.axes_cube(
            ren, x_bound=x_bound, y_bound=y_bound, z_bound=z_bound
        )
        assembly = vtk.vtkAssembly()
        assembly.AddPart(axes)
        return assembly

    @staticmethod
    def create_axis_widget(scale, xyzLabels):
        """
        Creates a widget that can be used with the VTK Orientation Widget

        """
        axes = vtk.vtkAxesActor()
        axes.SetScale(scale[0], scale[1], scale[2])
        axes.SetShaftTypeToCylinder()
        axes.SetXAxisLabelText(xyzLabels[0])
        axes.SetYAxisLabelText(xyzLabels[1])
        axes.SetZAxisLabelText(xyzLabels[2])
        axes.SetCylinderRadius(0.5 * axes.GetCylinderRadius())
        axes.SetConeRadius(1.025 * axes.GetConeRadius())
        axes.SetSphereRadius(1.5 * axes.GetSphereRadius())
        tprop = axes.GetXAxisCaptionActor2D().GetCaptionTextProperty()
        tprop.ItalicOn()
        tprop.ShadowOn()
        tprop.SetFontFamilyToTimes()
        # Use the same text properties on the other two axes.
        axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().ShallowCopy(tprop)
        axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().ShallowCopy(tprop)
        return axes
