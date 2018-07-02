#!/usr/bin/env python

import unittest

import rospy
from ros_speech2text.msg import transcript
from rpi_integration.htm_controller import HTMController


class TestHTMPlanner(unittest.TestCase):
    def setUp(self):
        rospy.loginfo("Initializing test controller")
        self.controller         = HTMController()
        self.root               = self.controller.htm.root

        self.param_prefix       = "/rpi_integration"

        self.top_down_queries   = rospy.get_param(self.param_prefix + '/top_down')
        self.bottom_up_queries  = rospy.get_param(self.param_prefix + '/bottom_up')
        self.horizontal_queries = rospy.get_param(self.param_prefix + '/horizontal')
        self.stationary_queries = rospy.get_param(self.param_prefix + '/stationary')

        self.subtasks = ['chair', 'seat', 'back-of-object', 'top-of-object']

    def test_all_top_down_queries(self):

        param_query_answers = [
        set(['In order to  BUILD CHAIR', 'First, we will BUILD BACK-OF-OBJECT',
         'Then, we will BUILD SEAT',
         'Finally, we will REQUEST-ACTION GIVE SCREWDRIVER']),
        set(['In order to  BUILD SEAT',
         'First, we will FASTEN ARTIFACT-LEGs TO SEAT',
         'Then, we will BUILD ARTIFACT-LEG', 'Then, we will BUILD ARTIFACT-LEG',
         'Then, we will BUILD ARTIFACT-LEG',
         'Finally, we will BUILD ARTIFACT-LEG']),
        set(['In order to  BUILD BACK-OF-OBJECT',
         'First, we will FASTEN TOP ARTIFACTs TO BACK-OF-OBJECT',
         'Then, we will FASTEN VERTICAL ARTIFACTs',
         'Finally, we will BUILD TOP-OF-OBJECT']),
        set(['In order to  BUILD TOP-OF-OBJECT',
         'First, we will FASTEN(brackets)',
         'Then, we will HOLD(dowel)',
         'Then, we will GET(dowel-top)',
         'Finally, we will Parallelized Subtasks of BUILD TOP-OF-OBJECT'])
        ]

        build_chair_answer = set([
            'Our task is to BUILD CHAIR',
            'First, we will BUILD BACK-OF-OBJECT', 'Then, we will BUILD SEAT',
            'Finally, we will REQUEST-ACTION GIVE SCREWDRIVER'
        ])

        for q in self.top_down_queries:
            if '{}' in q:
                for s,a in zip(self.subtasks, param_query_answers):
                    query    = q.format(s)
                    response = set(self.controller._select_query(query))
                    # rospy.loginfo(query)
                    # rospy.loginfo(response)
                    self.assertEqual(response, a)
            else:
                response = set(self.controller._select_query(q))
                self.assertEqual(response, build_chair_answer)

    def test_all_bottom_up_paramaterized(self):

        param_query_answers = [
        set(['In order to  BUILD CHAIR', 'First, we will BUILD BACK-OF-OBJECT',
         'Then, we will BUILD SEAT',
         'Finally, we will REQUEST-ACTION GIVE SCREWDRIVER']),
        set(['In order to  BUILD SEAT',
         'First, we will FASTEN ARTIFACT-LEGs TO SEAT',
         'Then, we will BUILD ARTIFACT-LEG', 'Then, we will BUILD ARTIFACT-LEG',
         'Then, we will BUILD ARTIFACT-LEG',
         'Finally, we will BUILD ARTIFACT-LEG']),
        set(['In order to  BUILD BACK-OF-OBJECT',
         'First, we will FASTEN TOP ARTIFACTs TO BACK-OF-OBJECT',
         'Then, we will FASTEN VERTICAL ARTIFACTs',
         'Finally, we will BUILD TOP-OF-OBJECT']),
        set(['In order to  BUILD TOP-OF-OBJECT',
         'First, we will FASTEN(brackets)',
         'Then, we will HOLD(dowel)',
         'Then, we will GET(dowel-top)',
         'Finally, we will Parallelized Subtasks of BUILD TOP-OF-OBJECT'])
        ]

        build_chair_answer = set([
            'Our task is to BUILD CHAIR',
            'First, we will BUILD BACK-OF-OBJECT', 'Then, we will BUILD SEAT',
            'Finally, we will REQUEST-ACTION GIVE SCREWDRIVER'
        ])

        for q in self.bottom_up_queries:
            if '{}' in q:
                for s,a in zip(self.subtasks, param_query_answers):
                    query    = q.format(s)
                    response = set(self.controller._select_query(query))
                    rospy.loginfo(query)
                    rospy.loginfo(response)
                    #self.assertEqual(response, a)
            # else:
            #     response = set(self.controller._select_query(q))
            #     # self.assertEqual(response, build_chair_answer)

    # def test_top_down(self):
    #     query_1            = "what is the task"
    #     query_1_responses  = self.controller._select_query(query_1)

    #     build_chair_answer = [
    #         'Our task is to BUILD CHAIR',
    #         'First, we will BUILD BACK-OF-OBJECT', 'Then, we will BUILD SEAT',
    #         'Finally, we will REQUEST-ACTION GIVE SCREWDRIVER'
    #     ]

    #     for i, j in zip(build_chair_answer, query_1_responses):
    #         self.assertEqual(i, j)



    def test_bottom_up_why(self):
        self.controller.curr_action = self.controller.htm.find_node_by_name(self.root,
                                                                            "seat")
        why_query             = "why are we doing this"
        why_query_conditional = "why"

        query_1_response      = self.controller._select_query(why_query)
        query_1_answer        = ['So that we can BUILD CHAIR']

        query_2_response      = self.controller._select_query(why_query_conditional)
        query_2_answer        = ['So that we can Start']

        self.controller.curr_action = self.controller.htm.find_node_by_name(self.root,
                                                                            "leg")
        rospy.loginfo(query_2_response)

        self.assertEqual(query_1_answer, query_1_response)
        self.assertEqual(query_2_answer, query_2_response)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('rpi_integration', 'test_htm_controller', TestHTMPlanner)
