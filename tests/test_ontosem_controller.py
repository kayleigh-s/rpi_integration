#!/usr/bin/env python
import unittest
import rospy
import requests

from rpi_integration.ontosem_controller import OntoSemController

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

class TestOntoSemController(unittest.TestCase):
    def setUp(self):
        rospy.loginfo("Initializing test ontosem controller")

        self.controller = OntoSemController()
        self.ontosem    = OntsemSim()

    def test_bootstrap(self):
        bootstap_dict = self.ontosem.GET_bootstrap()
        rospy.loginfo("BOOT STRAP DICT: ", bootstap_dict)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('rpi_integration', 'test_ontosem_controller', TestOntoSemController)
