import logging
import os
import random
import sys
import time
from multiprocessing import Process

import zmq
from zmq.log.handlers import PUBHandler

LOG_LEVELS = (
    logging.DEBUG,
    logging.INFO,
    logging.WARN,
    logging.ERROR,
    logging.CRITICAL,
)

LOGNAME = "smartscaffolding.log"


def log_worker(port, interval=1, level=logging.DEBUG):
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.connect("tcp://127.0.0.1:%i" % port)

    logger = logging.getLogger(str(os.getpid()))
    handler = PUBHandler(pub)

    logger.setLevel(level)

    logger.addHandler(handler)
    print("starting logger at %i with level=%s" % (os.getpid(), level))

    while True:
        print("Test")
        level = random.choice(LOG_LEVELS)
        logging.log(level, "Robot %i!" % os.getpid())
        time.sleep(interval)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        n = int(sys.argv[1])
    else:
        n = 1

    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    else:
        port = 5558

    level = logging.DEBUG
    logging.basicConfig(level=level)
    # start the log generators
    workers = [
        Process(
            target=log_worker, args=(port,), kwargs=dict(level=random.choice(LOG_LEVELS)),
        )
        for i in range(n)
    ]

    [w.start() for w in workers]

    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     [w.terminate() for w in workers]
