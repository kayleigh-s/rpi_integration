import json
import os
import re
import unittest
from io import open

import requests

addr = "http://0.0.0.0:5002"
center_idxpost_addr = "{}/learn".format(addr)
get_addr = "{}/alpha/gettree".format(addr)
post_addr = "{}/learn".format(addr)
headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
}


def post(data):
    req = requests.post(url=post_addr, data=data, headers=headers)
    return req


def get(frmt="json"):
    assert (frmt == "json" or frmt == "pretty")
    url = get_addr + "?format={}".format(frmt)
    req = requests.get(url=url)
    return req


def delete():
    url = addr + "/alpha/reset"
    requests.delete(url=url, headers=headers)


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

    def test_One(self):
        self._delete()
        data = '[["u", "We will build a chair."], ["a", "get-screwdriver"]]'
        self._learn(data)

        with open(
                os.path.join(self.curdir, "out/testOne.json"),
                "r",
                encoding="utf-8") as o:
            ground_truth = json.load(o)
            output = json.loads(self._get().text)
            # print self._get().text
            self.assertEqual(ground_truth, output)

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
        requests.delete(url=url, headers=headers)

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

        out_path = os.path.join(self.curdir, "out/", f)

        for a in args:
            self._learn(a)

        with open(out_path, "r") as o:
            ground_truth = json.load(o)
            output = json.loads(self._get().text)

            self.assertEqual(ground_truth, output)


# delete()
# post(data)
# g2 = get("json")
# text = g2.text
# text = re.sub("\"id\"\: \d+", "\"id\": ", text)
# print(text)
# print
# gt = json.loads(text)
delete()
with open("./in/leg_foot.json", "r") as i:
    print("loading")
    post(i)
    print(get().text)

if __name__ == '__main__':
    unittest.main()
