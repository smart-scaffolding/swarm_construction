import numpy as np


class BlueprintTemplate:
    def __init__(
        self,
        length=10,
        width=10,
        height=10,
        data=np.array([[[1] * 10] * 10] * 10),
        name="Undefined",
    ):
        self.name = name
        self.length = length
        self.width = width
        self.height = height
        self.data = data


class PlaygroundBlueprint(BlueprintTemplate):
    def __init__(self, name="Playground"):
        data = np.array(
            [
                [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
                [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
                [[1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0]],
                [[1, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
                [[1, 0, 0, 0], [1, 1, 1, 0], [1, 1, 1, 1]],
                [[1, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]],
            ]
        )
        super().__init__(data=data, length=6, width=3, height=4, name=name)


class Pyramid(BlueprintTemplate):
    def __init__(self, length, width, height, name="Pyramid"):

        if height < 4:
            height = 4

        blueprint_base = np.array([[[1] * 1] * length] * width)
        blueprint_status = list()
        blueprint_status.append(blueprint_base)
        blueprint_status.append(
            blueprint_base
        )  # append second time to have blocks above base (first layer)

        for i in range(height - 1):
            next_level = np.array([[[1] * 1] * length] * width)
            for block in range(i + 1):
                next_level[block, :, :] = 0
                next_level[(block + 1) * -1, :, :] = 0
                next_level[:, block, :] = 0
                next_level[:, (block + 1) * -1, :] = 0

            blueprint_status.append(next_level)

        data = np.dstack(blueprint_status)
        super().__init__(data=data, length=length, width=width, height=height, name=name)


class Cube(BlueprintTemplate):
    def __init__(self, length, width, height, name="Cube"):
        data = np.array([[[1] * height] * length] * width)
        super().__init__(data=data, length=length, width=width, height=height, name=name)


class Plane(BlueprintTemplate):
    def __init__(self, length, width, name="Plane"):
        data = np.array([[[1] * 1] * length] * width)
        super().__init__(data=data, length=length, width=width, height=1, name=name)


class BlockWorld(BlueprintTemplate):
    def __init__(self, length, width, height, name="BlockWorld"):

        blueprint_status = list()

        for i in range(height):
            next_level = np.array([[[0] * 1] * length] * width)
            for block in range(1):
                next_level[block, :, :] = 1
                next_level[(block + 1) * -1, :, :] = 1
                next_level[:, block, :] = 1
                next_level[:, (block + 1) * -1, :] = 1

            blueprint_status.append(next_level)

        data = np.dstack(blueprint_status).T
        super().__init__(data=data, length=length, width=width, height=height, name=name)


class RandomWorld(BlueprintTemplate):
    def __init__(self, length, width, height, name="RandomWorld", mu=0, sigma=1):
        data = np.random.normal(mu, sigma, (length, width, height))
        data[data >= 0.5] = 1
        data[data < 0.5] = 0

        super().__init__(data=data, length=length, width=width, height=height, name=name)


class RandomWorldConstrained(BlueprintTemplate):
    def __init__(
        self, length, width, height, name="RandomWorldConstrained", mu=0, sigma=1
    ):
        data = np.random.normal(mu, sigma, (length, width, height))
        data[data >= 0.5] = 1

        indices = np.argwhere(data < 0.5)
        for x, y, z in indices:
            if z == 0:
                data[x, y, z] = 1
            elif data[x, y, z - 1] == 1:
                data[x, y, z] = 0
            else:
                data[x, y, z] = 1

        super().__init__(data=data, length=length, width=width, height=height, name=name)


class StairwayToHeaven(BlueprintTemplate):
    def __init__(self, name="StairwayToHeaven"):
        data = np.load("../../blueprints/StairwayToHeaven.npy")
        super().__init__(length=12, width=12, height=11, data=data, name=name)


if __name__ == "__main__":
    # a = RandomWorldConstrained(10, 10, 5)
    # a = Pyramid(10, 10, 4)
    a = StairwayToHeaven()
    print(a.data.shape)
