#!/usr/bin/env python

import unittest

import rospy
from ros_speech2text.msg import transcript
from rpi_integration.htm_controller import HTMController


class TestHTMPlanner(unittest.TestCase):
    def setUp(self):
        #rospy.init_node("test")
        self.controller = HTMController()
        self.LISTEN_TOPIC = 'speech_to_text/transcript'
        self.speech_pub = rospy.Publisher(self.LISTEN_TOPIC, transcript, queue_size=10)

    def test_bottom_up(self):
        utter_1 = "what is the task"
        utter_2 = "how can i build a chair"

        trans = transcript()
        trans.transcript = utter_1

        #self.speech_pub.publish(trans)
        rospy.loginfo(self.controller._listen_query_cb(utter_1))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('rpi_integration', 'test_htm_controller', TestHTMPlanner)
