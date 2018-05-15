import json
import os
import re
import unittest
from io import open

import requests


class TestLearner(unittest.TestCase):
    def setUp(self):
        self.headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
        }

        self.addr = "http://0.0.0.0:5002"
        self.curdir = os.path.dirname(os.path.abspath(__file__))
        self.maxDiff = None

    def test_leg_foot(self):
        print("Testing leg...")
        self._compare_from_file("leg_foot.json")

    def test_all_actions(self):
        print("Testing all actions...")
        self._compare_from_file("all_actions.json")

    def test_full_chair(self):
        print("Testing full chair...")
        self._compare_from_file("full_chair.json")

    def test_online(self):
        d1 = '[["u", "We will build a leg."], ["a", "get-dowel"]]'
        d2 = '[["a", "get-bracket-foot]]'

        print("Testing oline learning...")
        self._compare_from_string("leg_foot.json", d1, d2)

    def _learn(self, data):
        url = self.addr + "/learn"
        req = requests.post(url=url, data=data, headers=self.headers)
        return req

    def _get(self, frmt="json"):
        assert (frmt == "json" or frmt == "pretty")
        url = self.addr + "/alpha/gettree?format={}".format(frmt)
        req = requests.get(url=url)
        return req

    def _delete(self):
        url = self.addr + "/alpha/reset"
        requests.delete(url=url, headers=self.headers)

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


if __name__ == '__main__':
    unittest.main()
