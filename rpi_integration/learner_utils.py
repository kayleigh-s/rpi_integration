#!/usr/bin/env python
import requests

HEADER = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
}


def learn(data, post_addr):
    """Sends a POST request to learn a new HTN (or update the previous one)
    from a sequence of actions and utterances."""
    req = requests.post(url=post_addr, data=data, headers=HEADER)
    return req


def get(get_addr):
    """Sends a GET request to retrieve the current HTN."""
    req = requests.get(url=get_addr)
    return req


def delete(delete_addr):
    """Sends a DELETE request to reset the learning."""
    requests.delete(url=delete_addr, headers=HEADER)
