import json
import os
import unittest
from io import open

import requests


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

    def _learn(self, data):
        req = requests.post(
            url=self.post_addr, data=data, headers=self.headers)
        return req

    def _get(self):
        req = requests.get(url=self.get_addr)
        return req

    def _delete(self):
        requests.delete(url=self.delete_addr, headers=self.headers)

    # Compares output of learning from input file to ground truth in output file
    def _compare_from_file(self, f):
        self._delete()

        in_path = os.path.join(self.curdir, "in/", f)
        out_path = os.path.join(self.curdir, "out/", f)

        with open(in_path, "r") as i, open(out_path, "r") as o:
            self._learn(i)

            ground_truth = json.load(o)
            output = json.loads(self._get().text)

            self.assertEqual(ground_truth, output)

    def _compare_from_string(self, f, *args):
        self._delete()
        for a in args:
            self._learn(a)

        out_path = os.path.join(self.curdir, "out/", f)

        with open(out_path, "r") as o:
            ground_truth = json.load(o)
            output = json.loads(self._get().text)

            print(ground_truth)
            print
            print(self._get().text)

            self.assertEqual(ground_truth, output)
