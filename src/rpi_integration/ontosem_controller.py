import rospy
import actionlib

# from face_recognition.msg import FRClientGoal
from face_recognition.msg import FaceRecognitionGoal, FaceRecognitionAction, FRClientGoal

from svox_tts.srv import Speech, SpeechRequest
from human_robot_collaboration.controller import BaseController
from rpi_integration.learner_utils import RESTOntoSemUtils
from ros_speech2text.msg import transcript


class OntoSemController(BaseController, RESTOntoSemUtils):
    RECOGNIZE_ONCE = 0
    RECOGNIZE_CONT = 1
    LEARN_FACE     = 2
    TRAIN_FACE     = 3
    DUMMY_STRING   = 'none'

    def __init__(self):
        self.param_prefix = "/rpi_integration"
        self.autostart    = rospy.get_param(self.param_prefix + '/autostart')
        self.use_stt      = rospy.get_param(self.param_prefix + '/use_stt', False)
        self.use_tts      = rospy.get_param(self.param_prefix + '/use_tts', True)

        self.client       = actionlib.SimpleActionClient("face_recognition", FaceRecognitionAction)
        self.goal         = FaceRecognitionGoal()

        self.storage_1    = rospy.get_param("action_provider/objects_left").values()
        self.storage_2    =  rospy.get_param("action_provider/objects_right").values()

        self._listen_sub        = rospy.Subscriber(self.STT_TOPIC, #self.SPEECH_SERVICE,
                                                   transcript, self._listen_query_cb)
        BaseController.__init__(
            self,
            use_left=True,
            use_right=True,
            use_stt=False,
            use_tts=self.use_tts,
            recovery=False,
        )

        RESTOntoSemUtils.__init__(self)

        rospy.logwarn("Waiting for actionserver....")
        self.client.wait_for_server()
        rospy.logwarn("Action server ready!")

        self._bootstrap()

    def _perceptual_update(self):
        visual_dict = {
            "storage-1": self.storage_1,
            "storage-2": self.storage_2,
            "faces": self._get_faces()
        }

        rospy.loginfo("DICT: {}".format(visual_dict))

        # self.POST_visible_objects(visual_dict)

    def _run(self):
        self._perceptual_update()

    def _bootstrap(self):
        "Sends OntoSem initial 'bootstrap' info about workspace"
        bootstrap_dict = {}
        bootstrap_dict.update(rospy.get_param("action_provider/objects_left"))
        bootstrap_dict.update(rospy.get_param("action_provider/objects_right"))

        faces = rospy.get_param(self.param_prefix + '/all_faces')
        bootstrap_dict['faces'] = faces

        rospy.loginfo(bootstrap_dict)

        # self.POST_bootstrap_ontosem(bootstrap_dict)

    def _active_cb(self):
        rospy.loginfo("GOAL IS ACTIVE")

    def _feedback_cb(self, fb):
        rospy.loginfo("GETTING FEEDBACK")
        if fb.order_id == self.RECOGNIZE_CONT:
            rospy.loginfo("FEEDBACK: {}, conf: {}".format(fb.names, fb.confidence))

        elif fb.order_id == self.LEARN_FACE:
            rospy.loginfo("LEARNING: {}".format(fb.names))

    def _done_cb(self, state, result):
        if result.order_id == self.RECOGNIZE_CONT or result.order_id == self.RECOGNIZE_ONCE:
            rospy.loginfo("FEEDBACK: {}, conf: {}".format(result.names, result.confidence))

        # elif result.order_id == self.LEARN_FACE:
        #     rospy.loginfo("LEARNING: {}".format(result.names))


    def _get_faces(self):
        "Tries to identify a face from the video stream for 3 seconds"
        self.goal.order_id = self.RECOGNIZE_ONCE
        self.goal.order_argument = self.DUMMY_STRING
        self.client.send_goal(self.goal, active_cb=self._active_cb,
                              feedback_cb=self._feedback_cb, done_cb = self._done_cb)
        self.client.wait_for_result(rospy.Duration(3.0))

        names = self.client.get_result()
        if names:
            return names.names
        else:
            rospy.loginfo("No faces were found!")
            return ['']

    def _listen_query_cb(self, msg):
        pass
