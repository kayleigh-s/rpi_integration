#!/usr/bin/env python
import requests
import rospy

def parse_action(act, obj_dict):
    """ Parses actions in rpi notation (e.g. GET(dowel)) and converts them
      into human_robot_collaboration compatible actions (e.g. get_pass [13, 25])

        :param act: the action in rpi notation
        :return: the command, the arm, the object to control
        """
    try:
        cmd, arm, obj = obj_dict[act].pop()
    except AttributeError:  # Can only pop if its a list
        cmd, arm, obj = obj_dict[act]

    return cmd, arm, obj


class RESTOntoSemUtils(object):
    """REST API for interfacing with learner"""
    def __init__(self):

        addr = "http://0.0.0.0:5002"
        self.GET_start_addr           = addr + "/iidea/start"
        self.POST_bootstrap_addr       = addr + "/yale/bootstrap"
        self.POST_visual_input_addr    = addr + "/yale/visual-input"
        self.POST_action_callback_addr = addr + "/iidea/callback"
        self.POST_verbal_command_addr  = addr + "/iidea/input"

        self.GET_chair_addr            = addr + "/bootstrap?package=backend.resources.experiments&resource=chair.knowledge"
        self.GET_robot_addr            = addr + "/bootstrap?package=backend.resources.experiments&resource=Robot_1.knowledge"
        self.GET_robot_command_addr    = addr + "/robotcommand"

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

    def GET_start(self):
        r = self._GET(self.GET_start_addr)
        rospy.loginfo("GETTING START REQUEST {}".format(r.text))

    def POST_bootstrap_ontosem(self, id_dict):
        r = self._POST(self.POST_bootstrap_addr, id_dict)
        rospy.loginfo(r.text)
        return r 

    def POST_visible_objects(self, obj_dict):
        return self._POST(self.POST_visual_input_addr, obj_dict)

    def POST_completed_action(self, act_dict):
        return self._POST(self.POST_action_callback_addr, act_dict)

    def POST_verbal_command(self, command_dict):
        return self._POST(self.POST_verbal_command_addr, command_dict)


    def GET_bootstrap(self):
        r1 = self._GET(self.GET_chair_addr)
        r2 = self._GET(self.GET_robot_addr)
        return r1, r2



class RESTLearnerUtils(object):
    def __init__(self):
        """REST API for interfacing with learner"""
        addr = "http://0.0.0.0:5002"

        self.learn_addr  = addr + "/learn"
        self.delete_addr = addr + "/alpha/reset"
        self.get_addr    = addr + "/alpha/gettree?format =json"
        self.query_addr  = addr + "/query"

        self.HEADER      = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
        }

    def learn(self, data):
        """Sends a POST request to learn a new HTN (or update the previous one)
        from a sequence of actions and utterances."""
        req = requests.post(url=self.learn_addr, data=data, headers=self.HEADER)
        return req

    def get(self):
        """Sends a GET request to retrieve the current HTN."""
        req = requests.get(url=self.get_addr)
        return req

    def delete(self):
        """Sends a DELETE request to reset the learning."""
        requests.delete(url=self.delete_addr, headers=self.HEADER)

    def query(self, data):
        """Sends a POST request to update the previous one
        from a sequence of utterances."""
        req = requests.post(url=self.query_addr, data=data, headers=self.HEADER)
        return req

if __name__ == '__main__':

    r1 = requests.post(url="http://localhost:5003/ontology/manage/remote/download",
                  data={"ontology":"robot-v.1.0.0"})

    requests.post(url="http://localhost:5003/ontology/manage/local/install",
                  data={"ontology":"robot-v.1.0.0"})

    requests.post (url="http://localhost:5003/ontology/manage/activate",
                   data={"ontology":"robot-v.1.0.0"})


    print(r1.text)
