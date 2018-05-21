#!/usr/bin/env python
import requests


class RESTUtils(object):
    def __init__(self):
        """REST API for interfacing with learner"""
        addr = "http://0.0.0.0:5002"

        self.post_addr = addr + "/learn"
        self.delete_addr = addr + "/alpha/reset"
        self.get_addr = addr + "/alpha/gettree?format=json"

        self.HEADER = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
        }

    def learn(self, data):
        """Sends a POST request to learn a new HTN (or update the previous one)
        from a sequence of actions and utterances."""
        req = requests.post(url=self.post_addr, data=data, headers=self.HEADER)
        return req

    def get(self):
        """Sends a GET request to retrieve the current HTN."""
        req = requests.get(url=self.get_addr)
        return req

    def delete(self):
        """Sends a DELETE request to reset the learning."""
        requests.delete(url=self.delete_addr, headers=self.HEADER)
