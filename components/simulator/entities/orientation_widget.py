from components.simulator.entities.object_creator import ObjectFactory
from components.simulator.model.graphics import MakeAxesActor
import vtk


class OrientationWidget:
    def __init__(self, iren, scale, xyzLabels):
        self.om2 = None
        axes = MakeAxesActor(scale, xyzLabels)

        print(axes)
        om2 = vtk.vtkOrientationMarkerWidget()
        om2.SetOrientationMarker(axes)

        # Position lower right in the viewport.
        om2.SetViewport(0.8, 0, 1.0, 0.2)
        om2.SetInteractor(iren)
        om2.EnabledOn()
        om2.InteractiveOn()
        print("Created orientation widget")
        self.om2 = om2
