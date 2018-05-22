#!/usr/bin/env python
import argparse
import ast
import os

from task_models.json_to_htm import json_to_htm

import rospy
from human_robot_collaboration.controller import BaseController
from rpi_integration.learner_utils import RESTUtils, parse_action
from std_msgs.msg import String

parser = argparse.ArgumentParser("Streams data from input file to learner")
parser.add_argument(
    '--path',
    help='path to the model files',
    default=os.path.join(
        os.path.abspath(os.path.dirname(__file__)),
        "../tests/in/full_chair.json"))


class StreamingController(RESTUtils, BaseController):

    BRING = 'get_pass'
    HOLD_TOP = 'hold_top'
    HOLD_LEG ='hold_leg'

    ACTION_DICT = {
        "get-seat": (BRING, BaseController.LEFT, 198),
        "get-back": (BRING, BaseController.LEFT, 201),
        "get-dowel":  [(BRING, BaseController.LEFT, 150), (BRING, BaseController.LEFT, 151),
                       (BRING, BaseController.LEFT, 152), (BRING, BaseController.LEFT, 153),
                       (BRING, BaseController.LEFT, 154), (BRING, BaseController.LEFT, 155)],
        "get-top-dowel": (BRING, BaseController.LEFT, 156),
        "get-bracket-foot": [(BRING, BaseController.RIGHT, 10),(BRING, BaseController.RIGHT, 11),
                              (BRING, BaseController.RIGHT, 12), (BRING, BaseController.RIGHT, 13)],
        "get-bracket-front":[(BRING, BaseController.RIGHT, 14),(BRING, BaseController.RIGHT, 15),
                             (BRING, BaseController.RIGHT, 22), (BRING, BaseController.RIGHT, 23)],
        "get-top-bracket":[(BRING, BaseController.RIGHT, 16), (BRING, BaseController.RIGHT, 17)],
        "get-bracket-back-right": (BRING, BaseController.RIGHT, 18),
        "get-bracket-back-left": (BRING, BaseController.RIGHT, 19),
        "get-screwdriver": (BRING, BaseController.RIGHT, 20),
        "hold-top-dowel": (HOLD_LEG, BaseController.RIGHT, 0),
        "hold-dowel": (HOLD_LEG, BaseController.RIGHT, 0),
        "hold-back": (HOLD_LEG, BaseController.RIGHT, 0),
        "hold-seat": (HOLD_LEG, BaseController.RIGHT, 0)
    }

    def __init__(self, infile):
        """
        Trains learner incrementally from file and sends actions to robot as they occur.
        """
        super(RESTUtils, self).__init__()
        super(BaseController, self).__init__(left=True, right=True, speech=False,
                                             listen=True, recovery=True)
        "Publishes learning incrementally from by streaming data from input file"

        self.infile = infile

        rospy.init_node('streaming_pub')
        self.learner_pub = rospy.Publisher(
            'web_interface/json', String, queue_size=10)

    def run(self):
        self.delete() # delete any left over learning

        with open(self.infile, 'r') as i:
            cmds = ast.literal_eval(i.read()) # transform string input into list
            cmds_iter = iter(cmds) # Allows finer control over how we access cmds
            try:
                while not self.finished:
                    # Wait for voice to be recognized
                    if self.listen_sub.wait_for_msg(timeout=20.):
                        line = cmds.next() # Get line of data
                        data_type = line[0] # action or utterance?
                        data = line[1] # What was done/said?

                        # Format data for learner and post
                        for_learner = '[[\"{}\", \"{}\"]]'.format(data_type, data)
                        self.learn(cmd)

                        # If data is action, have robot do it!
                        if data_type == 'a':
                            cmd, arm, obj = self.parse_action(data, self.OBJECT_DICT)
                            self._action(arm, (cmd, [obj]), {'wait': True})

                        # Publish results of learning
                        htm_so_far = get(self.get_addr).text
                        self.learner_pub.publish(htm_so_far)
            except StopIteration: # Happens once we reach end of iter
                return



args = parser.parse_args()
streamer = StreamingController(args.path)

try:
    streamer.run()
except KeyboardInterrupt():
    rospy.signal_shutdown("controller shutdown")
