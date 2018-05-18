import unittest

from test_learner_utils import TestLearnerUtils


class TestLearner(TestLearnerUtils):

    def test_leg_foot(self):
        self._compare_from_file("leg_foot.json")

    def test_all_actions(self):
        self._compare_from_file("all_actions.json")

    def test_full_chair(self):
        self._compare_from_file("full_chair.json")

    # def test_online(self):
    #     d1 = '[["u", "We will build a leg."], ["a", "get-dowel"]]'
    #     d2 = '[["a", "get-bracket-foot"]]'

    #     print("Testing online learning...")
    #     self._compare_from_string("leg_foot.json", d1, d2)


if __name__ == '__main__':
    unittest.main()
