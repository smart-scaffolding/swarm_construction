import pandas
from pkg_resources import resource_filename

"""
SCRIPT USED TO ANALYZE DEBUGGING RESULTS FROM debug.py script
"""

FILENAME = "simulator/logs/simulator_debug.csv"

file_location = resource_filename("components", FILENAME)

df = pandas.read_csv(file_location)

print("*" * 60)
print("\n")
print(
    df.groupby("FunctionID", sort=False).agg({"Duration": ["mean", "sum", "min", "max"]})
)
print("\n")
print("*" * 60)
