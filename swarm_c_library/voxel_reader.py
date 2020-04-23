import numpy as np
import pandas as pd

"""
Use following link to upload obj, be sure to download it as a text file

https://drububu.com/miscellaneous/voxelizer/?out=txt
"""

FILE = "/Users/calebwagner/Downloads/tower.txt"
SAVE_LOCATION = "/Users/calebwagner/SmartScaffoldingMQP_Code/swarm_construction/blueprints/tower"


data = pd.read_csv(FILE, header=None)
data.columns = ["x", "y", "z"]

x, y, z = data.max()

blueprint = np.zeros((x + 1, y + 1, z + 1))
for index, row in data.iterrows():
    blueprint[row["x"], row["y"], row["z"]] = 1

print(blueprint.shape)
np.save(SAVE_LOCATION, blueprint)