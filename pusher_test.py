from pusher import Pusher
from random import random, randint, choice
from datetime import datetime
import time

pusher = Pusher(
    app_id="882780",
    key="b19a4591cdd9ad1d70f7",
    secret="eb1cdfa23df02681662e",
    cluster="us2",
    ssl=True,
)

count = 0
while True:
    current_time = datetime.now().strftime("%H:%M:%S")
    pusher.trigger(
        "robot",
        "state",
        {
            "id": "ROBOT_1",
            "position": (
                f"{random()*10:.1f}",
                f"{random()*10:.1f}",
                f"{random()*10:.1f}",
                "top",
            ),
            "angles": [
                f"{random()*360:.1f}",
                f"{random()*360:.1f}",
                f"{random()*360:.1f}",
            ],
            "grippers": {"a": randint(0, 100), "d": randint(0, 100)},
            "battery": 77,
            "blocks_placed": count,
            "a_link_blocks": 1,
            "d_link_blocks": 0,
            "robot_state": "MOVING",
            "end_effector_velocity": {"label": current_time, "value": randint(0, 100)},
        },
    )

    blueprint = choice(
        [
            "EmpireStateBuilding",
            "Temple",
            "TajMahal",
            "Colosseum",
            "Cube_10x10x10",
            "Playground",
            "StarTrek",
            "asdfd",
        ]
    )

    pusher.trigger(
        "homeblock",
        "state",
        {
            "robot_count": randint(0, 10),
            "building_dimensions": "(10, 10, 10)",
            "blocks_to_place": randint(0, 100),
            "blueprint": blueprint,
        },
    )
    count += 1
    time.sleep(randint(1, 7))
