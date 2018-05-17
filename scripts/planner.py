from random import shuffle

from task_models.json_to_htm import json_to_htm

#from human_robot_collaboration.controller import BaseController


# OBJECT_DICT = {
#     "GET(seat)": (BaseController.LEFT, 198),
#     "GET(BACK)": (BaseController.LEFT, 201),
#     "GET(dowel)": [(BaseController.LEFT, 150), (BaseController.LEFT, 151),
#                    (BaseController.LEFT, 152), (BaseController.LEFT, 153),
#                    (BaseController.LEFT, 154), (BaseController.LEFT, 155)],
#     "GET(dowel-top)": (BaseController.LEFT, 156),
#     "GET(FOOT_BRACKET)": [(BaseController.RIGHT, 10),(BaseController.RIGHT, 11),
#                          (BaseController.RIGHT, 12), (BaseController.RIGHT, 13)],
#     "GET(bracket-front)": [(BaseController.RIGHT, 14),(BaseController.RIGHT, 15)],
#     "GET(bracket-front)": [(BaseController.RIGHT, 16), (BaseController.RIGHT, 17)]
#     "GET(bracket-back-right)": (BaseController.RIGHT, 18),
#     "GET(bracket-back-left)": (BaseController.RIGHT, 19),
#     "GET(screwdriver)": (BaseController.RIGHT, 20),
# }

path = "../tests/out/full_chair.json"

class HTMController():
    "Controls Baxter using HTN derived from json"
    def __init__(self, json_path):
        self.htm = json_to_htm(path)


    @property
    def robot_actions(self):
        return list(self._get_actions(self.htm.root))

    def _get_actions(self, root):
        "Recursively retrieves actions in correct order"
        name = root.name
        kind = root.kind

        try:
            children = root.children
            # If parallel, then permute the action orders
            if kind == 'Parallel':
                shuffle(children)

            for child in children:
                for c in self._get_actions(child):
                    yield c
        except ValueError:
            if root.action.agent == 'ROBOT':
                yield name



controller = HTMController(path)
print controller.robot_actions
