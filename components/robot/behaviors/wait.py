##############################################################################
# Imports
##############################################################################

import py_trees
import time
from components.robot.communication.messages import StatusUpdateMessage
from components.robot.common.states import RobotBehaviors
##############################################################################
# Classes
##############################################################################


class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name, status_identifier, robot_communicator, robot_state="robot_status",):
        super().__init__(name=name)
        self.communicator = robot_communicator.robot_communicator
        self.robot_id = robot_communicator.robot_id
        self.state = self.attach_blackboard_client("State", "state")
        self.status_identifier = status_identifier
        self.keys = {
            "robot_state": robot_state,
        }
        self.state.register_key(key=robot_state, access=py_trees.common.Access.WRITE)

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        robot_status_key = self.keys["robot_state"]
        self.robot_status = self.state.get(robot_status_key)

    def update(self):
        new_status = py_trees.common.Status.RUNNING


        if self.robot_status != self.status_identifier:
            print(f"[{self.name.upper()}]: Returning success {self.robot_status} {self.status_identifier}")
            return py_trees.common.Status.SUCCESS


        print(f"[{self.name.upper()}]: Waiting...")
        response_message = StatusUpdateMessage(status=self.status_identifier, payload="Waiting...")
        self.communicator.send_communication(topic=self.robot_id, message=response_message)
        return new_status

def create__waiting_root(robot_communicator):
    wait_action = py_trees.decorators.RunningIsFailure(child=Wait(name="Wait", status_identifier=RobotBehaviors.WAIT,
                                                                  robot_communicator=robot_communicator))


    root = py_trees.composites.Selector(name="Root")

    root.add_children([wait_action])
    return root

##############################################################################
# Main
##############################################################################


def main():

    py_trees.logging.level = py_trees.logging.Level.INFO

    root = create__waiting_root()


    behaviour_tree = py_trees.trees.BehaviourTree(root)




    writer = py_trees.blackboard.Client(name="Writer")
    writer.register_key(key="state/robot_status", access=py_trees.common.Access.WRITE)
    writer.set(name="state/robot_status", value=RobotBehaviors.WAIT)



    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    # py_trees.console.read_single_keypress()
    for unused_i in range(1, 25):
        try:
            behaviour_tree.tick()
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

if __name__ == '__main__':
    main()