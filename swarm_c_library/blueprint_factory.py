from collections import defaultdict

from swarm_c_library.defined_blueprints import *


class BluePrintFactory:
    def __init__(self):
        self.possible_blueprints = defaultdict(BlueprintTemplate)
        self.initialize_blueprints()

    def initialize_blueprints(self):
        self.possible_blueprints["Playground"] = PlaygroundBlueprint()
        self.possible_blueprints["Pyramid_10x10x4"] = Pyramid(
            10, 10, 4, name="Pyramid_10x10x4"
        )
        self.possible_blueprints["Pyramid_10x10x5"] = Pyramid(
            10, 10, 5, name="Pyramid_10x10x5"
        )
        self.possible_blueprints["Pyramid_20x20x10"] = Pyramid(
            20, 20, 10, name="Pyramid_20x20x10"
        )
        self.possible_blueprints["Cube_10x10x10"] = Cube(10, 10, 10, name="Cube_10x10x10")
        self.possible_blueprints["Cube_20x20x10"] = Cube(20, 20, 10, name="Cube_20x20x10")
        self.possible_blueprints["Cube_5x5x5"] = Cube(5, 5, 5, name="Cube_5x5x5")
        self.possible_blueprints["Plane_5x5x1"] = Plane(5, 5, name="Plane_5x5x1")
        self.possible_blueprints["Plane_10x10x1"] = Plane(10, 10, name="Plane_10x10x1")
        self.possible_blueprints["Plane_12x12x1"] = Plane(12, 12, name="Plane_12x12x1")
        self.possible_blueprints["Plane_20x20x1"] = Plane(20, 20, name="Plane_20x20x1")
        self.possible_blueprints["Plane_20x20x2"] = Plane(20, 20, name="Plane_20x20x2", height=2)
        self.possible_blueprints["BlockWorld_4x4x10"] = BlockWorld(
            10, 10, 4, name="BlockWorld_4x4x10"
        )
        self.possible_blueprints["BlockWorld_1x1x10"] = BlockWorld(
            10, 10, 1, name="BlockWorld_1x1x10"
        )
        self.possible_blueprints["BlockWorld_3x3x10"] = BlockWorld(
            10, 10, 3, name="BlockWorld_3x3x10"
        )
        self.possible_blueprints["RandomWorld_10x10x3"] = RandomWorld(
            10, 10, 3, name="RandomWorld_10x10x3"
        )
        self.possible_blueprints["RandomWorld_10x10x5"] = RandomWorld(
            10, 10, 5, name="RandomWorld_10x10x5"
        )
        self.possible_blueprints[
            "RandomWorldConstrained_10x10x5"
        ] = RandomWorldConstrained(10, 10, 3, name="RandomWorldConstrained_10x10x5")
        self.possible_blueprints["StairwayToHeaven"] = StairwayToHeaven()

    def get_blueprint(self, blueprint_label):
        return self.possible_blueprints[blueprint_label]


if __name__ == "__main__":
    a = BluePrintFactory()
    print(a.possible_blueprints["StairwayToHeaven"].data)
