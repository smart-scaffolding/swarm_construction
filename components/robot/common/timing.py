from collections import defaultdict
from time import time
import threading
from queue import Queue
from csv import DictWriter
from datetime import datetime, date
import components.robot.config as config
import json
from copy import copy


class BehaviorTiming:
    def __init__(self, behaviors, filename, write_data_interval=10):
        self.behaviors = defaultdict(lambda: 0)
        for behavior in behaviors:
            self.behaviors[behavior] = 0
        self.behaviors["Timestamp"] = datetime.now()
        self.filename = filename
        self.start_time = time()
        self.elapsed_time = self.start_time
        with open(filename, "a", newline="") as csvfile:
            fieldnames = self.behaviors.keys()
            writer = DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
        self.data_queue = Queue()
        self.write_data_interval = write_data_interval
        self.time_writer = WriteTimeToFile(filename, data_queue=self.data_queue)
        self.last_write = self.start_time

    def start_behavior_timing(self):
        self.time_writer.start()

    def update_time(self, behavior, parent=None):
        current_time = time()
        if parent:
            self.behaviors[parent] += current_time - self.elapsed_time
        else:
            self.behaviors[behavior] += current_time - self.elapsed_time

        self.elapsed_time = current_time

        print(behavior)
        print(parent)
        config.pusher.trigger(
            "robot",
            "behavior_tree",
            {"behavior": behavior, "parent": parent, "id": config.ROBOT_ID},
        )
        # print("Updated time")
        if (int(time() - self.last_write)) >= self.write_data_interval:
            self.behaviors["Timestamp"] = datetime.now()
            self.data_queue.put(self.behaviors)
            # print("Putting in queue")
            self.last_write = time()


class WriteTimeToFile(threading.Thread):
    def __init__(self, filename, data_queue):
        super(WriteTimeToFile, self).__init__(daemon=True)
        self.filename = filename
        self.data_queue = data_queue

    def run(self):
        for dict_to_write in iter(self.data_queue.get, None):
            with open(self.filename, "a", newline="") as csvfile:
                fieldnames = dict_to_write.keys()
                writer = DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow(dict_to_write)
                # print("Wrote to file")
            dict_to_write_pusher = copy(dict_to_write)
            dict_to_write_pusher["id"] = config.ROBOT_ID
            config.pusher.trigger(
                "robot",
                "behavior_time",
                json.dumps(dict_to_write_pusher, default=default),
            )


def default(o):
    if isinstance(o, (date, datetime)):
        return o.isoformat()


if __name__ == "__main__":
    from random import randint
    from time import sleep

    behaviors = [
        "MoveToBlockToRemove",
        "RemoveBlock",
        "MoveToPlaceBlock",
        "PlaceBlock",
    ]
    behaviors_no_parents = [
        "wait",
        "update",
        "move",
        "ferry",
        "build",
        ]
    parents = ["wait", "update", "move", "ferry", "build", None, None, None, None, None]


    timer = BehaviorTiming(
        behaviors=behaviors, filename="../results/test.csv", write_data_interval=3
    )
    timer.start_behavior_timing()
    while True:
        rand_index_parents = randint(0, len(parents) - 1)
        parent = parents[rand_index_parents]
        if parent is None:
            rand_index = randint(0, len(behaviors_no_parents) - 1)
            timer.update_time(behaviors_no_parents[rand_index])
        else:
            rand_index = randint(0, len(behaviors) - 1)
            timer.update_time(behaviors[rand_index], parent=parent)
        sleep(randint(0, 5))
