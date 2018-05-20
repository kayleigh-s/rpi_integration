import argparse
import threading
import os
import time
import rospy
from random import shuffle

from task_models.json_to_htm import json_to_htm

from human_robot_collaboration.controller import BaseController
from human_robot_collaboration.service_request import finished_request


parser = argparse.ArgumentParser("Run the autonomous HTM controller")
parser.add_argument(
    '--path', help='path to the model files',
    default=os.path.join(os.path.abspath(os.path.dirname(__file__)),
                         "../tests/out/full_chair.json"))


class HTMController(BaseController):
    "Controls Baxter using HTN derived from json"

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
        "HOLD(seat)": (HOLD_TOP, BaseController.RIGHT, 0),
        "HOLD(back)": (HOLD_TOP, BaseController.RIGHT, 0)

    }

    def __init__(self, json_path):
        self.htm = json_to_htm(json_path)
        self.prev_arm = None
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
        return list(self._get_actions(self.htm.root))


    def take_action(self,act):
        cmd, arm, idx = self._parse_action(act)
        arm_str = "LEFT" if arm == 0 else 'RIGHT'
        same_arm = self.prev_arm == arm
        elapsed_time = time.time() - self.strt_time
        #rospy.loginfo("Num threads {}".format(threading.current_thread()))

        # if self.prev_arm == arm:
        #     rospy.loginfo("Waiting on last action..")
        #     self.last_r.wait_result()
        #     rospy.loginfo("Last Action finished: {}".format(self.last_r.finished))
        # if self.prev_arm == arm and arm == BaseController.LEFT:
        #     rospy.loginfo("Waiting on left..")
        #     self._last_action_right_request.wait_result()
        # elif self.prev_arm == arm and arm == BaseController.RIGHT:
        #     rospy.loginfo("Waiting on right..")
        #     self._last_action_left_request.wait_result()
        # if self.prev_arm == None:
        #     self.prev_arm = arm

        # while(not self.last_r.finished):
        #     rospy.loginfo("NOT FINISHED")
        #     rospy.sleep(.1)

        rospy.loginfo("Taking action {} on object {} with {} arm at time {}".format(cmd,
                                                                                    idx,
                                                                                    arm_str,
                                                                                    elapsed_time))
        rospy.loginfo("Same arm: {}".format(self.prev_arm == arm))
        self.prev_arm = arm
        return self._action(arm, (cmd, [idx]), {'wait': False})



    def take_actions(self):
        last_r = finished_request
        prev_arm = 2
        for a in self.robot_actions:
            cmd, arm, idx = self._parse_action(a)
            arm_str = "LEFT" if arm == 0 else 'RIGHT'
            same_arm = prev_arm == arm
            elapsed_time = time.time() - self.strt_time

            # if same_arm:
            #     try:
            #         for t in threading.enumerate():
            #             t.join()
            #     except RuntimeError:
            #         pass

            rospy.loginfo("Same arm: {}".format(prev_arm == arm))
            # if prev_arm == arm:
            #     rospy.loginfo("Waiting on last action..")
            #     last_r.wait_result()

            last_r = self._action(arm, (cmd, [idx]), {'wait': False})
            print(
                "Taking action {} on object {} with {} arm at time {}".format(cmd,
                                                                              idx,
                                                                              arm_str,
                                                                              elapsed_time))
            prev_arm = arm


    def _run(self):
        rospy.loginfo('Starting autonomous control')
        self.take_actions()
        # for a in self.robot_actions:
        #     r = self.take_action(a)

    def _get_actions(self, root):
        "Recursively retrieves actions in correct order"
        name = root.name
        kind = root.kind

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

    def _parse_action(self, act):

        try:
            cmd, arm, idx = self.OBJECT_DICT[act].pop()
        except AttributeError: # Can only pop if its a list
            cmd, arm, idx = self.OBJECT_DICT[act]

        return cmd, arm, idx

try:
    args = parser.parse_args()
    print(args.path)
    controller = HTMController(args.path)

    controller.run()
except rospy.ROSInterruptException, KeyboardInterrupt:
    pass
