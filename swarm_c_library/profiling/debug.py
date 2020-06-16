import logging
import time
from collections import defaultdict

from pkg_resources import resource_filename


class FunctionTimerDebugger:
    """
    Utility used to time a specific funtion and log results to a CSV

    """
    def __init__(self):
        self.function_profiles = defaultdict(lambda: [])

        LOGFILE = resource_filename("components", "simulator/logs/simulator_debug.csv")

        self.logger = logging.getLogger("myapp")
        hdlr = logging.FileHandler(LOGFILE)
        formatter = logging.Formatter("%(message)s")
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr)
        self.logger.setLevel(logging.DEBUG)
        self.logger.debug(f"FunctionID,Call,Duration")

    def update_time(self, function_id, duration):
        history = self.function_profiles[function_id]

        new_val = (len(history) + 1, duration)
        history.append(new_val)
        self.function_profiles[function_id] = history
        # print(self.function_profiles)
        self.logger.debug(f"{function_id},{new_val[0]},{new_val[1]}")

    def __del__(self):
        print(self.function_profiles)


ftd = FunctionTimerDebugger()


def timefunc(function_id):
    def inner(f):
        def timer(*args, **kwargs):
            start = time.time()

            result = f(*args, **kwargs)
            end = time.time()

            duration = end - start

            ftd.update_time(function_id, duration)
            return result

        return timer

    return inner
