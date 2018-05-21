import ast
import json
import os
import time
import unittest
import numpy as np
from io import open

import requests
from task_models.json_to_htm import json_to_htm
from rpi_integration.learner_utils import RESTUtils


class TestLearnerUtils(unittest.TestCase):
    def setUp(self):
        self.headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
        }

        addr = "http://0.0.0.0:5002"

        self.curdir = os.path.dirname(os.path.abspath(__file__))
        self.maxDiff = None
        self.rest_utils = RESTUtils()


    def _compare_from_file(self, f):
        """Compares output of learning from input file
        to ground truth in output file."""
        print("Testing " + f + "...")
        self.rest_utils.delete()

        in_path = os.path.join(self.curdir, "in/", f)
        out_path = os.path.join(self.curdir, "out/", f)

        with open(in_path, "r") as i, open(out_path, "r") as o:
            # print(i.read())
            self.rest_utils.learn(i)

            ground_truth = json.load(o)
            output = json.loads(self.rest_utils.get().text)

            self.assertEqual(ground_truth, output)

    def _compare_from_string(self, f, *args):
        print("Testing " + f + "...")
        self.rest_utils.delete()

        for a in args:
            self.rest_utils.learn(a, )

        out_path = os.path.join(self.curdir, "out/", f)

        with open(out_path, "r") as o:
            ground_truth = json.load(o)
            output = json.loads(self.rest_utils.get().text)

            self.assertEqual(ground_truth, output)

    def _compare_htm_to_file(self, f):
        self.rest_utils.delete()

        out_path = os.path.join(self.curdir, "out/", f)
        htm = json_to_htm(out_path)

        with open(out_path, "r") as o:
            ground_truth = json.load(o)
            output = json.loads(json.dumps(htm.as_dictionary()))

            self.assertEqual(ground_truth, output)

    def _compare_from_file_streaming(self, f):
        self.rest_utils.delete()

        in_path = os.path.join(self.curdir, "in/", f)
        out_path = os.path.join(self.curdir, "out/", f)

        with open(in_path, 'r') as i, open(out_path, 'r') as o:
            ground_truth = json.load(o)

            cmds = ast.literal_eval(i.read())
            for c in cmds:
                cmd = '[[\"{}\", \"{}\"]]'.format(c[0], c[1])
                self.rest_utils.learn(cmd, )

            print(self.rest_utils.get().text)
            output = json.loads(self.rest_utils.get().text)

            self.assertEqual(ground_truth, output)
