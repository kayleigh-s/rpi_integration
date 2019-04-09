#!/usr/bin/env python
import unittest
import rospy
import requests

from rpi_integration.ontosem_controller import OntoSemController

PKG  = 'rpi_integration'
NAME = 'test_ontosem_controller'


class OntsemSim(object):
    def __init__(self):
        "Simulates HTTP messages from ontosem"

        addr                          = "http://0.0.0.0:5002"

        self.GET_bootstrap_addr       = addr + "/yale/bootstrap"
        self.GET_visual_input_addr    = addr + "/yale/visual-input"
        self.GET_action_callback_addr = addr + "/iidea/callback"
        self.GET_verbal_command_addr  = addr + "/iidea/input"
        self.POST_robot_command_addr  = addr + "/robotcommand"

        self.HEADER                    = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
        }

        def _POST(self, addr, data):
            req = requests.post(url=addr, data=data, headers=self.HEADER)
            return req

        def _GET(self, addr):
            """Sends a GET request to retrieve the current HTN."""
            req = requests.get(url=addr)
            return req

        # Here I've simply inverted RESTOntoSemUtils methods
        # from POSTs to GETS (and vice versa) so that we can send
        # arbitrary messages to the controller

        def GET_bootstrap(self):
            return self._GET(self.GET_bootstrap_addr, id_dict)

        def GET_visible_objects(self):
            return self._GET(self.GET_visual_input_addr, obj_dict)

        def GET_completed_action(self):
            return self._GET(self.GET_action_callback_addr, act_dict)

        def GET_verbal_command(self):
            return self._GET(self.GET_verbal_command_addr, command_dict)

        def POST_robot_command(self, cmd_dict):
            return self._POST(self.POST_robot_command_addr, cmd_dict)

# See https://wiki.ros.org/unittest for more info
class TestOntoSemController(unittest.TestCase):

    def setUp(self):
        rospy.loginfo("Initializing test ontosem controller")

        self.controller = OntoSemController()
        self.ontosem    = OntsemSim()

    def test_bootstrap(self):

        gt_bootstrap_dict = {} # TODO: Fill this out
        bootstap_dict     = self.ontosem.GET_bootstrap()

        self.assertEqual(gt_bootstrap_doct, bootstap_dict)

    def test_actions(self):
        "Tests for correct callback after action"

        cmd1 = {"get": "198", "callback":"SELF.CALLBACK.1"} # seat
        cmd2 = {"get": "10", "callback":"SELF.CALLBACK.2"} # foot_1
        cmds = [cmd1, cmd2]

        cb1  = {"callback-id": "SELF.CALLBACK.1"}
        cb2  = {"callback-id": "SELF.CALLBACK.2"}
        cbs  = [cb1, cb2]

        for cmd, cb in zip(cmds, cbs):
            self.ontosem.POST_robot_command_addr(cmd) # Send command to controller
            self.assertEqual(self.ontosem.GET_completed_action(), cb)

    # TODO Implement (this one is important)
    def test_perceptual_update(self):
        raise NotImplementedError

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestOntoSemController)
