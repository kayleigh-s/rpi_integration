import argparse
import json

import requests

parser = argparse.ArgumentParser("Run the Learner on json training files")
# parser.add_argument('path', help='path to the model files')
parser.add_argument('-l', '--list', help='delimited list input', type=str)

addr = "http://0.0.0.0:5002"
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


# delete()
# post(data)
# g2 = get("json")
# text = g2.text
# text = re.sub("\"id\"\: \d+", "\"id\": ", text)
# print(text)
# print
# gt = json.loads(text)
args = parser.parse_args()
delete()
fs = args.list.split(",")
print(fs)
for f in fs:
    with open(f, "r") as i:
        post(i)
print(get().text)
