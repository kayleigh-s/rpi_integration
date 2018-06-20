#!/usr/bin/env python

import unittest

import rospy
from ros_speech2text.msg import transcript
from rpi_integration.htm_controller import HTMController


class TestHTMPlanner(unittest.TestCase):
    def setUp(self):
        rospy.loginfo("Initializing test controller")
        self.controller = HTMController()
        self.root = self.controller.htm.root

    def test_top_down(self):
        query_1            = "what is the task"
        query_1_responses  = self.controller._select_query(query_1)

        build_chair_answer = [
            'Our task is to BUILD CHAIR',
            'First, we will BUILD BACK-OF-OBJECT', 'Then, we will BUILD SEAT',
            'Finally, we will REQUEST-ACTION GIVE SCREWDRIVER'
        ]

        for i, j in zip(build_chair_answer, query_1_responses):
            self.assertEqual(i, j)

    def test_parameterized_top_down_queieres(self):
        query_seat                  = "how can we build a seat"
        query_back                  = "how can we build a back-of-object"
        query_screwdriver           = "how can we build a screwdriver"

        # Responses and answers are sets to account for random order
        # due to them being parallelized tasks
        query_seat_responses        = set(self.controller._select_query(query_seat))
        query_back_responses        = set(self.controller._select_query(query_back))
        query_screwdriver_responses = set(self.controller._select_query(query_screwdriver))

        seat_answer        = set([
            'In order to  BUILD SEAT',
            'First, we will FASTEN ARTIFACT-LEGs TO SEAT',
            'Then, we will BUILD ARTIFACT-LEG',
            'Then, we will BUILD ARTIFACT-LEG',
            'Then, we will BUILD ARTIFACT-LEG',
            'Finally, we will BUILD ARTIFACT-LEG'
        ])

        back_answer        =  set([
            'In order to  BUILD BACK-OF-OBJECT',
            'First, we will FASTEN TOP ARTIFACTs TO BACK-OF-OBJECT',
            'Then, we will FASTEN VERTICAL ARTIFACTs',
            'Finally, we will BUILD TOP-OF-OBJECT'
        ])
        screwdriver_answer = set([
            'In order to  REQUEST-ACTION GIVE SCREWDRIVER',
            'All we need to do is GET(screwdriver)'
        ])

        self.assertEqual(query_seat_responses, seat_answer)
        self.assertEqual(query_back_responses, back_answer)
        self.assertEqual(query_screwdriver_responses, screwdriver_answer)


    def test_bottom_up_queries(self):
        self.controller.curr_action = self.controller.htm.find_node_by_name(self.root,
                                                                            "seat")
        rospy.loginfo(self.controller.curr_action)
        why_query = "why are we doing this"
        why_query_conditional = "why"

        query_1_response = self.controller._select_query(why_query)
        query_1_answer   = ['So that we can BUILD CHAIR']

        query_2_response = self.controller._select_query(why_query_conditional)
        query_2_answer   = ['So that we can Start']

        self.controller.curr_action = self.controller.htm.find_node_by_name(self.root,
                                                                            "leg")
        rospy.loginfo(query_2_response)

        self.assertEqual(query_1_answer, query_1_response)
        self.assertEqual(query_2_answer, query_2_response)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('rpi_integration', 'test_htm_controller', TestHTMPlanner)
