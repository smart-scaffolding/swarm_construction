from queue import PriorityQueue
from components.structure.communication.messages import MoveToPointMessage

import numpy as np
def distance(me, other):
    dist = (other[0] - me[0])**2 + (other[1] - me[1])**2
    return dist

def robots_distances_to_locations(robots, points):

    for robot in robots:
        robot.closest_points = []
        for point in points:
            dist = distance(robot.pos, point.pos)
            robot.closest_points.append((dist, point))
        robot.closest_points = sorted(robot.closest_points, key=lambda point: point[1].pos,reverse=True)
        closest_points = np.array(robot.closest_points)[:, 0]
        arg_min_score = np.argmin(closest_points)
        robot.desired_target = robot.closest_points[arg_min_score][1]
        robot.desired_target_distance = robot.closest_points[arg_min_score][0]

def assign_robots_closest_point(robots, points, robot_communicator, level):
    robots_distances_to_locations(robots, points)
    q = PriorityQueue()
    for robot in robots:
        q.put((-robot.desired_target_distance, robot))

    # sorted_robots = sorted(robots, key=lambda bot: bot.desired_target_distance, reverse=True)
    print(30*"-")
    print("CLAIMING ROBOTS")
    claimed = []
    while not q.empty():
        dist, robot = q.get()
        index = points.index(robot.desired_target)
        if points[index] not in claimed:
            robot.target = robot.desired_target
            robot.claimed_division = robot.desired_target
            # points[index].status = "CLAIMED"
            claimed.append(points[index])
            print(f"Robot {robot.id} at: {robot.pos}")
            print("\t" + 30 * "-")
            print(f"\tClaimed target (id={robot.target.id}): {robot.target.pos}")
            print(f"\t\tDistance: {robot.desired_target_distance}")
            print("\n")

            tolerance = 2
            if abs(robot.pos[0] - robot.target.pos[0]) > tolerance and abs(robot.pos[1] - robot.target.pos[1]) > \
                    tolerance:
                robot.pos = (robot.target.pos[0], robot.target.pos[1]) #TODO: Uncomment this and let robots determine new
                # position
                if robot_communicator:
                    robot_communicator.send_communication(topic=robot.id, message=MoveToPointMessage(
                        destination=(robot.target.pos[0], robot.target.pos[1], level)))
            else:
                print("No need for robot to move, as already there")

        else:
            try:
                robot.find_new_target(points)
                q.put((-robot.desired_target_distance, robot))
            except IndexError:
                print("Something went wrong, unable to find new target")
                print("Points")
                print(points)
                print(robot)
                raise IndexError

if __name__ == '__main__':
    # robots = [Robot(1, (0, 0)), Robot(2, (0, 1)), Robot(4, (0, 2)), Robot(3, (1, 1))]
    # points = [Location(1, (0, 0)), Location(2, (0, 1)), Location(5, (1, 1)), Location(6, (1, 2))]
    #
    # # robots_distances_to_locations(robots, points)
    # assign_robots_closest_point(robots, points)
    pass