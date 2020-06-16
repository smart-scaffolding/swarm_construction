import vtk

from components.simulator.entities.gui.axis_entity import AxisFactory


class OrientationWidget:
    """
    Used to create a widget showing the axis of orientation for the simulator.
    
    """

    def __init__(self, iren, scale, xyzLabels):
        self.widget = None
        axes = AxisFactory.create_axis_widget(scale, xyzLabels)

        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)

        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(iren)
        om2.EnabledOn()
        om2.InteractiveOn()
        self.widget = om2
