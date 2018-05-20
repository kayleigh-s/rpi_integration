import ast
import json
import os
import time
import unittest
import numpy as np
from io import open

import requests
from task_models.json_to_htm import json_to_htm
from rpi_integration.learner_utils import learn, get, delete


class TestLearnerUtils(unittest.TestCase):
    def setUp(self):
        self.headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
        }

        addr = "http://0.0.0.0:5002"

        self.curdir = os.path.dirname(os.path.abspath(__file__))
        self.post_addr = addr + "/learn"
        self.delete_addr = addr + "/alpha/reset"
        self.get_addr = addr + "/alpha/gettree?format=json"
        self.maxDiff = None


    def _compare_from_file(self, f):
        """Compares output of learning from input file
        to ground truth in output file."""
        print("Testing " + f + "...")
        delete(self.delete_addr)

        in_path = os.path.join(self.curdir, "in/", f)
        out_path = os.path.join(self.curdir, "out/", f)

        with open(in_path, "r") as i, open(out_path, "r") as o:
            # print(i.read())
            learn(i, self.post_addr)

            ground_truth = json.load(o)
            output = json.loads(get(get_addr).text)

            self.assertEqual(ground_truth, output)

    def _compare_from_string(self, f, *args):
        print("Testing " + f + "...")
        delete(self.delete_addr)

        for a in args:
            learn(a, self.post_addr)

        out_path = os.path.join(self.curdir, "out/", f)

        with open(out_path, "r") as o:
            ground_truth = json.load(o)
            output = json.loads(get(get_addr).text)

            self.assertEqual(ground_truth, output)

    def _compare_htm_to_file(self, f):
        delete(self.delete_addr)

        out_path = os.path.join(self.curdir, "out/", f)
        htm = json_to_htm(out_path)

        with open(out_path, "r") as o:
            ground_truth = json.load(o)
            output = json.loads(json.dumps(htm.as_dictionary()))

            self.assertEqual(ground_truth, output)

    def _compare_from_file_streaming(self, f):
        delete(self.delete_addr)

        in_path = os.path.join(self.curdir, "in/", f)
        out_path = os.path.join(self.curdir, "out/", f)

        with open(in_path, 'r') as i, open(out_path, 'r') as o:
            ground_truth = json.load(o)

            cmds = ast.literal_eval(i.read())
            for c in cmds:
                cmd = '[[\"{}\", \"{}\"]]'.format(c[0], c[1])
                learn(cmd, self.post_addr)

            print(get(get_addr).text)
            output = json.loads(get(get_addr).text)

            self.assertEqual(ground_truth, output)
