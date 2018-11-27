import rospy

from svox_tts.srv import Speech, SpeechRequest
from human_robot_collaboration.controller import BaseController
from rpi_integration.learner_utils import RESTOntoSemUtils
from ros_speech2text.msg import transcript


class OntoSemController(BaseController, RESTOntoSemUtils):
    def __init__(self):
        self.param_prefix       = "/rpi_integration"
        self.autostart          = rospy.get_param(self.param_prefix + '/autostart')
        self.use_stt            = rospy.get_param(self.param_prefix + '/use_stt', False)
        self.use_tts            = rospy.get_param(self.param_prefix + '/use_tts', True)

        BaseController.__init__(
            self,
            use_left=True,
            use_right=True,
            use_stt=False,
            use_tts=self.use_tts,
            recovery=False,
        )

