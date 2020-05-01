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
                [[1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0]],
                [[1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0]],
                [[1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0]],
                [[1, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0], [1, 1, 1, 1, 0, 0]],
                [[1, 0, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0], [1, 1, 1, 1, 0, 0]],
                [[1, 0, 0, 0, 0, 0], [1, 1, 1, 1, 0, 0], [1, 1, 1, 1, 0, 0]],
            ]
        )
        super().__init__(data=data, length=6, width=3, height=6, name=name)


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
    def __init__(self, length, width, height, name="Cube", pad=4):
        data = np.array([[[1] * height] * length] * width)
        data = pad_blueprint(data, pad)
        super().__init__(
            data=data,
            length=length + pad,
            width=width + pad,
            height=height + pad,
            name=name,
        )


class Plane(BlueprintTemplate):
    def __init__(self, length, width, name="Plane", height=1, pad=4):
        data = np.array([[[1] * height] * length] * width)
        data = pad_blueprint(data, pad)
        super().__init__(
            data=data,
            length=length + pad,
            width=width + pad,
            height=height + pad,
            name=name,
        )


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
    def __init__(self, length, width, height, name="RandomWorld", mu=0, sigma=1, pad=4):
        data = np.random.normal(mu, sigma, (length, width, height))
        data[data >= 0.5] = 1
        data[data < 0.5] = 0
        data = pad_blueprint(data, pad)
        super().__init__(
            data=data,
            length=length + pad,
            width=width + pad,
            height=height + pad,
            name=name,
        )


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
    def __init__(self, name="StairwayToHeaven", pad=4):
        data = np.load("../blueprints/StairwayToHeaven.npy")
        data = pad_blueprint(data, pad)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Torus(BlueprintTemplate):
    # 1064 blocks
    # (24, 24, 24)
    def __init__(self, name="Torus", pad=0):
        data = np.load("../blueprints/torus.npy")
        data = pad_blueprint(data, pad, y=17)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Castle(BlueprintTemplate):
    # 4436 blocks
    # (39, 39, 62)
    def __init__(self, name="Castle", pad=0):
        data = np.load("../blueprints/castle.npy")
        data = pad_blueprint(data, pad, y=13)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Church(BlueprintTemplate):
    # 88509 blocks
    # (151, 151, 151)
    def __init__(self, name="Church", pad=0):
        data = np.load("../blueprints/church.npy")
        data = pad_blueprint(data, pad, y=45)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class TajMahal(BlueprintTemplate):
    # 17494 blocks
    # (57, 57, 57)
    def __init__(self, name="Church", pad=0):
        data = np.load("../blueprints/tajmahal.npy")
        data = pad_blueprint(data, pad, y=24)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class StarTrek(BlueprintTemplate):
    # 2170 blocks
    # (25, 25, 40)
    def __init__(self, name="StarTrek", pad=0):
        data = np.load("../blueprints/startrek.npy")
        data = pad_blueprint(data, pad, y=13)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Temple(BlueprintTemplate):
    # 2529 blocks
    # (20, 20, 28)
    def __init__(self, name="Temple", pad=0):
        data = np.load("../../blueprints/temple.npy")
        data = np.swapaxes(data, 1, 2)
        data = np.flip(data, 1)
        data = pad_blueprint(data, pad, x=5, y=2)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Colosseum(BlueprintTemplate):
    # 3843 blocks
    # (29, 29, 29)
    def __init__(self, name="Colosseum", pad=0):
        data = np.load("../blueprints/colosseum.npy")
        data = pad_blueprint(data, pad, y=20)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class EmpireStateBuilding(BlueprintTemplate):
    def __init__(self, name="EmpireStateBuilding", pad=0):
        data = np.load("../../blueprints/empire.npy")
        data = pad_blueprint(data, pad)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class MQPLogo(BlueprintTemplate):
    # 16912 blocks
    # (221, 8, 18)
    def __init__(self, name="MQP_Logo", pad=0):
        data = np.load("../blueprints/mqp_logo.npy")
        data = pad_blueprint(data, pad)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Building(BlueprintTemplate):
    # 1397 blocks
    # (15 22 15)
    def __init__(self, name="MQP_Logo", pad=0):
        data = np.load("../../blueprints/building.npy")
        data = data[47:62, :, 47:62]
        data = pad_blueprint(data, pad, x=7)
        x, y, z = data.shape
        # print(x, y, z)
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Cottage(BlueprintTemplate):
    # 1945 blocks
    # (15 15 16)
    def __init__(self, name="Cottage", pad=0):
        data = np.load("../../blueprints/cottage.npy")
        data = data[33:48, 21:34, 122:138]
        # data = np.flip(data, 0)
        data = pad_blueprint(data, pad, y=2)
        x, y, z = data.shape
        # print(x, y, z)
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Sofa(BlueprintTemplate):
    # 963 blocks
    # (23 23 10)
    def __init__(self, name="Sofa", pad=0):
        data = np.load("../../blueprints/sofa.npy")
        data = data[:, 2:, :]
        data = pad_blueprint(data, pad, y=18)
        x, y, z = data.shape
        # print(x, y, z)
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Bed(BlueprintTemplate):
    # 222 blocks
    # (10 10 5)
    def __init__(self, name="Bed", pad=0):
        data = np.load("../../blueprints/bed.npy")
        data = np.flip(data, 1)

        data = pad_blueprint(data, pad, x=2)
        x, y, z = data.shape
        # print(x, y, z)
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Camaro(BlueprintTemplate):
    # 663 blocks
    # (10 10 25)
    def __init__(self, name="Camaro", pad=0):
        data = np.load("../../blueprints/camaro.npy")
        data = data[:, :, :]
        data = pad_blueprint(data, pad, y=3)
        x, y, z = data.shape
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Tower(BlueprintTemplate):
    # 117 blocks
    # (5 5 9)
    def __init__(self, name="Tower", pad=0):
        data = np.load("../../blueprints/tower.npy")
        data = data[:, :, :]
        data = pad_blueprint(data, pad)
        data = np.swapaxes(data, 1, 2)

        x, y, z = data.shape
        print(f"Blueprint [{name}]: {x, y, z}")

        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class Thor(BlueprintTemplate):
    # 443 blocks
    # (12 7 21)
    # Division size: 4
    def __init__(self, name="Thor", pad=0):
        data = np.load("../../blueprints/thor.npy")
        data = data[:, :, :]
        data = np.swapaxes(data, 1, 2)
        data = pad_blueprint(data, pad, y=5)
        x, y, z = data.shape

        print(f"Blueprint [{name}]: {x, y, z}")
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


class StatueOfLiberty(BlueprintTemplate):
    # 697 blocks
    # (11 9 34)
    # Division size 10
    def __init__(self, name="StatueOfLiberty", pad=0):
        data = np.load("../../blueprints/liberty.npy")
        data = data[:, :, :]
        data = np.swapaxes(data, 1, 2)
        data = pad_blueprint(data, pad, y=2)

        x, y, z = data.shape
        print(f"Blueprint [{name}]: {x, y, z}")
        super().__init__(
            length=x, width=y, height=z, data=data, name=name
        )


def pad_blueprint(blueprint, pad, x=0, y=0, z=0):
    pad_x_before = 0
    pad_x_after = pad + x
    pad_y_before = 0
    pad_y_after = pad + y
    pad_z_before = 0
    pad_z_after = pad + z
    return np.pad(
        blueprint,
        (
            (pad_x_before, pad_x_after),
            (pad_y_before, pad_y_after),
            (pad_z_before, pad_z_after),
        ),
        "constant",
    )


if __name__ == "__main__":
    # a = RandomWorldConstrained(10, 10, 5)
    # a = Pyramid(10, 10, 4)
    # a = StairwayToHeaven()
    # a = Torus()
    # a = EmpireStateBuilding()
    a = TajMahal()
    print(a.data.shape)
