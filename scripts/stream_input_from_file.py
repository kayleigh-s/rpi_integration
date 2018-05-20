#!/usr/bin/env python
import argparse
import ast
import rospy
import os

from task_models.json_to_htm import json_to_htm
from std_msgs.msg import String

from rpi_integration.learner_utils import learn, get, delete

parser = argparse.ArgumentParser("Streams data from input file to learner")
parser.add_argument(
    '--path',
    help='path to the model files',
    default=os.path.join(
        os.path.abspath(os.path.dirname(__file__)),
        "../tests/in/full_chair.json"))


class InputStream(object):
    def __init__(self, infile):
        "Publishes learning incrementally from by streaming data from input file"

        addr = "http://0.0.0.0:5002"

        self.infile = infile
        self.post_addr = addr + "/learn"
        self.delete_addr = addr + "/alpha/reset"
        self.get_addr = addr + "/alpha/gettree?format=json"

        rospy.init_node('streaming_pub')
        self.learner_pub = rospy.Publisher(
            'learning_stream', String, queue_size=10)

    def run(self):
        delete(self.delete_addr) # delete any left over learning

        with open(self.infile, 'r') as i:
            cmds = ast.literal_eval(i.read()) # transform string input into list

            for c in cmds:
                cmd = '[[\"{}\", \"{}\"]]'.format(c[0], c[1])
                learn(cmd, self.post_addr)

                htm_so_far = get(self.get_addr).text
                self.learner_pub.publish(htm_so_far)


args = parser.parse_args()
streamer = InputStream(args.path)

try:
    streamer.run()
except KeyboardInterrupt():
    rospy.signal_shutdown("controller shutdown")
