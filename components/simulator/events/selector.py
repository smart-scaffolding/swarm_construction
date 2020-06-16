import vtk


class Selector:
    """
    Use this to highlight a selected component a certain color
    """

    def __init__(self, pipeline):
        self.LastPickedActor = None
        self.LastPickedProperty = vtk.vtkProperty()
        self.pipeline = pipeline

    def execute(self, obj, event):
        clickPos = self.pipeline.iren.GetEventPosition()

        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.pipeline.ren)

        # get the new
        self.NewPickedActor = picker.GetActor()

        if self.NewPickedActor:
            # Reset property of last thing selected
            if self.LastPickedActor:
                self.LastPickedActor.GetProperty().DeepCopy(self.LastPickedProperty)
            if self.NewPickedActor == self.LastPickedActor:
                self.NewPickedActor.GetProperty().DeepCopy(self.LastPickedProperty)
                return

            # Save property of picked actor
            self.LastPickedProperty.DeepCopy(self.NewPickedActor.GetProperty())

            # Highlight picked actor by changing its properties
            self.NewPickedActor.GetProperty().SetColor(1.0, 0.0, 0.0)
            self.NewPickedActor.GetProperty().SetDiffuse(1.0)
            self.NewPickedActor.GetProperty().SetSpecular(0.0)

            # save the last picked actor
            self.LastPickedActor = self.NewPickedActor

        return
