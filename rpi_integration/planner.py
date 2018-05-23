import argparse
import threading
import os
import time
import rospy
from random import shuffle

from task_models.json_to_htm import json_to_htm

from human_robot_collaboration.controller import BaseController
from human_robot_collaboration.service_request import finished_request

from rpi_integration.learner_utils import parse_action


parser = argparse.ArgumentParser("Run the autonomous HTM controller")
parser.add_argument(
    '--path', help='path to the model files',
    default=os.path.join(os.path.abspath(os.path.dirname(__file__)),
                         "../tests/out/full_chair.json"))


class HTMController(BaseController):
    """Controls Baxter using HTN derived from json"""

    strt_time = time.time()

    BRING = 'get_pass'
    HOLD_TOP = 'hold_top'
    HOLD_LEG ='hold_leg'

    OBJECT_DICT = {
        "GET(seat)": (BRING, BaseController.LEFT, 198),
        "GET(back)": (BRING, BaseController.LEFT, 201),
        "GET(dowel)": [(BRING, BaseController.LEFT, 150), (BRING, BaseController.LEFT, 151),
                       (BRING, BaseController.LEFT, 152), (BRING, BaseController.LEFT, 153),
                       (BRING, BaseController.LEFT, 154), (BRING, BaseController.LEFT, 155)],
        "GET(dowel-top)": (BRING, BaseController.LEFT, 156),
        "GET(FOOT_BRACKET)": [(BRING, BaseController.RIGHT, 10),(BRING, BaseController.RIGHT, 11),
                              (BRING, BaseController.RIGHT, 12), (BRING, BaseController.RIGHT, 13)],
        "GET(bracket-front)": [(BRING, BaseController.RIGHT, 14),(BRING, BaseController.RIGHT, 15),
                               (BRING, BaseController.RIGHT, 22), (BRING, BaseController.RIGHT, 23)],
        "GET(bracket-top)": [(BRING, BaseController.RIGHT, 16), (BRING, BaseController.RIGHT, 17)],
        "GET(bracket-back-right)": (BRING, BaseController.RIGHT, 18),
        "GET(bracket-back-left)": (BRING, BaseController.RIGHT, 19),
        "GET(screwdriver)": (BRING, BaseController.RIGHT, 20),
        "HOLD(dowel)": (HOLD_LEG, BaseController.RIGHT, 0),
        "HOLD(seat)": (HOLD_LEG, BaseController.RIGHT, 0),
        "HOLD(back)": (HOLD_LEG, BaseController.RIGHT, 0)

    }

    def __init__(self, json_path):
        self.htm = json_to_htm(json_path)
        self.last_r = finished_request
        super(HTMController, self).__init__(
            left=True,
            right=True,
            speech=False,
            listen=True,
            recovery=True,
        )

    @property
    def robot_actions(self):
        return self._get_actions(self.htm.root)

    def _take_actions(self):
        prev_arm = None # was left or right arm used previously?
        same_arm = True # Was previous action taken using the same arm as curr action?
        prev_same_arm = True # Was the previous previous action taken using same arm?

        for a in self.robot_actions:
            cmd, arm, obj = parse_action(a, self.OBJECT_DICT)
            arm_str = "LEFT" if arm == 0 else 'RIGHT'

            prev_same_arm = same_arm == prev_same_arm
            same_arm = True if prev_arm == None else prev_arm == arm

            rospy.loginfo("same arm {}, last same arm {}".format(same_arm,prev_same_arm))

            # Only sleep when encountering diff arms for first time.
            # prevents both arms from acting simultaneously.
            if  not same_arm and prev_same_arm:
                rospy.sleep(7)

            elapsed_time = time.time() - self.strt_time
            rospy.loginfo(
                "Taking action {} on object {} with {} arm at time {}".format(cmd,
                                                                              obj,
                                                                              arm_str,
                                                                              elapsed_time))
            # Send action to the robot
            self._action(arm, (cmd, [obj]), {'wait': False})

            elapsed_time = time.time() - self.strt_time
            rospy.loginfo(
                "Took action {} on object {} with {} arm at time {}".format(cmd,
                                                                            obj,
                                                                            arm_str,
                                                                            elapsed_time))
            prev_arm = arm


    def _run(self):
        rospy.loginfo('Starting autonomous control')
        rospy.sleep(3) # Add a little delay for self-filming!
        self._take_actions()

    def _get_actions(self, root):
        """ Recursively retrieves actions in correct order """
        name = root.name
        kind = root.kind # we're not evil

        try:
            children = root.children
            # If parallel, then permute the action orders
            if kind == 'Parallel':
                shuffle(children)

            # loop through all children and get their actions
            for child in children:
                for c in self._get_actions(child):
                    yield c
        except ValueError:
            if root.action.agent == 'robot':
                yield name


try:
    args = parser.parse_args()
    print(args.path)
    controller = HTMController(args.path)

    controller.run()
except rospy.ROSInterruptException, KeyboardInterrupt:
    pass
