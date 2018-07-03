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

        self.subtasks = ['chair', 'seat', 'back-of-object', 'top-of-object'] # NOT exhaustive

    def test_all_top_down_queries(self):

        # Ground truth
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
            # checks if query is paramaterized
            if '{}' in q:
                for s,a in zip(self.subtasks, param_query_answers):
                    query    = q.format(s)
                    response = set(self.controller._select_query(query))

                    self.assertEqual(response, a)
            else:
                response = set(self.controller._select_query(q))
                self.assertEqual(response, build_chair_answer)

    def test_bottom_up_paramaterized(self):

        # Ground truth
        param_query_answers   = [
            ['We are building a chair in order to Start'],
            ['We are building a seat in order to BUILD CHAIR'],
            ['We are building a back-of-object in order to BUILD CHAIR'],
            ['We are building a top-of-object in order to BUILD BACK-OF-OBJECT']
        ]


        why_query_answer              = ['So that we can Start']
        why_query_back_subtask_answer = ['So that we can BUILD CHAIR']

        for q in self.bottom_up_queries:
            if '{}' in q:
                for s,a in zip(self.subtasks, param_query_answers):
                    query    = q.format(s)
                    response = self.controller._select_query(query)

                    self.assertEqual(response, a)

                    # This bit the contextual 'why' query
                    if 'chair' not in s:
                        response = self.controller._select_query('why')

                        rospy.loginfo("ASKING WHY: {}".format(s))
                        rospy.loginfo(response)

                        # this one is a sub-task of build back-of-object
                        if 'top-of-object' in s:
                            self.assertEqual(response, why_query_back_subtask_answer)
                        else:
                            self.assertEqual(response, why_query_answer)





if __name__ == '__main__':
    import rostest
    rostest.rosrun('rpi_integration', 'test_htm_controller', TestHTMPlanner)
