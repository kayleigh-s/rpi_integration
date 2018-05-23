#!/usr/bin/env python
import argparse
import ast
import os
from threading import Lock

from task_models.json_to_htm import json_to_htm

import rospy
from human_robot_collaboration.controller import BaseController
from human_robot_collaboration.subscribers import CommunicationSubscriber
from ros_speech2text.msg import transcript
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

    WEB_TOPIC = '/web_interface/pressed'

    OBJECT_DICT = {
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
        RESTUtils.__init__(self)
        BaseController.__init__(self, left=True, right=True, speech=False,
                                             listen=True, recovery=True)
        self._lock = Lock()
        self.infile = infile

        self.learner_pub = rospy.Publisher('web_interface/json', String, queue_size=10)

        # Gets button presses from web interface.
        self._web_sub = rospy.Subscriber(self.WEB_TOPIC, String, self._web_cb)
        self._listen_sub = rospy.Subscriber(self.LISTEN_TOPIC, transcript, self._listen_cb)

        # Track if we have received a msg from above topics
        self._web_flag = False
        self._listen_flag = False


    def run(self):
        self.delete() # delete any left over learning

        with open(self.infile, 'r') as i:
            cmds = ast.literal_eval(i.read()) # transform string input into list
            cmds_iter = iter(cmds) # Allows finer control over how we access cmds
            try:

                while not self.finished:
                    if self._listen_flag or self._web_flag:

                        rospy.loginfo("Utterance or button press received")
                        line = cmds_iter.next() # Get line of data
                        data_type = line[0] # action or utterance?
                        data = line[1] # What was done/said?

                        # Format data for learner and post
                        for_learner = '[[\"{}\", \"{}\"]]'.format(data_type, data)
                        rospy.loginfo("Sending to leaner: {}".format(for_learner))
                        self.learn(data)

                        # If data is action, have robot do it!
                        if data_type == 'a':
                            cmd, arm, obj = parse_action(data, self.OBJECT_DICT)

                            rospy.loginfo("Taking action {} on object {}".format(cmd, obj))
                            self._action(arm, (cmd, [obj]), {'wait': True})

                        # Publish results of learning
                        htm_so_far = self.get().text
                        self.learner_pub.publish(htm_so_far)

                        # reset flags and in order to get next msg
                        with self._lock:
                            self._web_flag = False
                            self._listen_flag = False

            except StopIteration: # Happens once we reach end of iter
                return

    def _web_cb(self, msg):
        with self._lock:
            self._web_flag = True

    def _listen_cb(self, msg):
        with self._lock:
            self._listen_flag = True



args = parser.parse_args()
streamer = StreamingController(args.path)


try:
    streamer.run()
except KeyboardInterrupt():
    rospy.signal_shutdown("controller shutdown")
