import json
import re
import unittest
import os
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

    def test_One(self):
        self._delete()
        data = '[["u", "We will build a chair."], ["a", "get-screwdriver"]]'
        self._learn(data)

        with open(os.path.join(self.curdir,"out/testOne.json"), "r", encoding="utf-8") as o:
            ground_truth = json.load(o)
            output = json.loads(self._get().text)
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


#data = '[["u", "We will build a chair."], ["a", "get-screwdriver"]]'

#print(get("json").text)

# delete()
# post(data)
# g2 = get("json")
# text = g2.text
# text = re.sub("\"id\"\: \d+", "\"id\": ", text)
# print(text)
# print
# gt = json.loads(text)
# with open("./out/test.json", "r", encoding='utf-8') as f:
#     print("loading")
#     data = json.load(f)
#     print(data)
#     print(data == gt)
#print(data == gt)

if __name__ == '__main__':
    unittest.main()
