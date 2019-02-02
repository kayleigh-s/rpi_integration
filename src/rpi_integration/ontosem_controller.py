import rospy
import actionlib
import time

# Documentation here: http://wiki.ros.org/face_recognition
from face_recognition.msg import FaceRecognitionGoal, FaceRecognitionAction, FRClientGoal

from svox_tts.srv import Speech, SpeechRequest
from human_robot_collaboration.controller import BaseController
from rpi_integration.learner_utils import RESTOntoSemUtils, parse_action
from ros_speech2text.msg import transcript # message format for ros_speech2text


# TODO: include this somewhere else?
OBJECT_DICT = {
    "GET(seat)":                (   BRING, BaseController.LEFT, 198),
    "GET(back)":                (   BRING, BaseController.LEFT, 201),
    "GET(dowel)":              [(   BRING, BaseController.LEFT, 150),
                                (   BRING, BaseController.LEFT, 151),
                                (   BRING, BaseController.LEFT, 152),
                                (   BRING, BaseController.LEFT, 153),
                                (   BRING, BaseController.LEFT, 154),
                                (   BRING, BaseController.LEFT, 155)],
    "GET(dowel-top)":           (   BRING, BaseController.LEFT, 156),
    "GET(FOOT_BRACKET)":       [(   BRING, BaseController.RIGHT, 10),
                                (   BRING, BaseController.RIGHT, 11),
                                (   BRING, BaseController.RIGHT, 12),
                                (   BRING, BaseController.RIGHT, 13)],
    "GET(bracket-front)":      [(   BRING, BaseController.RIGHT, 14),
                                (   BRING, BaseController.RIGHT, 15),
                                (   BRING, BaseController.RIGHT, 22),
                                (   BRING, BaseController.RIGHT, 23)],
    "GET(bracket-top)":        [(   BRING, BaseController.RIGHT, 16),
                                (   BRING, BaseController.RIGHT, 17)],
    "GET(bracket-back-right)":  (   BRING, BaseController.RIGHT, 18),
    "GET(bracket-back-left)":   (   BRING, BaseController.RIGHT, 19),
    "GET(screwdriver)":         (   BRING, BaseController.RIGHT, 20),
    "HOLD(dowel)":              (HOLD_LEG, BaseController.RIGHT,  0),
    "HOLD(seat)":               (HOLD_LEG, BaseController.RIGHT,  0),
    "HOLD(back)":               (HOLD_LEG, BaseController.RIGHT,  0)

}


class OntoSemController(BaseController, RESTOntoSemUtils):
    """
    Sends and receives commands from OntoSem cogntiive architecture to Baxter robot
    """

    # These are used by face recognition software
    RECOGNIZE_ONCE = 0
    RECOGNIZE_CONT = 1
    LEARN_FACE     = 2
    TRAIN_FACE     = 3
    DUMMY_STRING   = 'none'

    def __init__(self):
        # Get the parameters from launchfile
        self.param_prefix = "/rpi_integration"
        self.autostart    = rospy.get_param(self.param_prefix + '/autostart')
        self.use_stt      = rospy.get_param(self.param_prefix + '/use_stt', False)
        self.use_tts      = rospy.get_param(self.param_prefix + '/use_tts', True)

        self.client       = actionlib.SimpleActionClient("face_recognition", FaceRecognitionAction)
        self.goal         = FaceRecognitionGoal()

        self.storage_1    = rospy.get_param("action_provider/objects_left").values()
        self.storage_2    = rospy.get_param("action_provider/objects_right").values()

        self.strt_time    = time.time()
        self.LISTENING    = False

        # Listens for speech commands from microphone
        self._listen_sub  = rospy.Subscriber(self.STT_TOPIC, #self.SPEECH_SERVICE,
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
        """
        POSTs the current contents of the workspace to OntoSem.
        """
        visual_dict = {
            "storage-1": self.storage_1,
            "storage-2": self.storage_2,
            "faces":     self._get_faces()
        }


        # TODO uncomment when running OntoSem
        # self.POST_visible_objects(visual_dict)

    # TODO Implement this function
    def _run(self):
        """
        Waits for commands from OntoSem, enacts them, and then send the appropriate
        updates back to OntoSem.
        """

        # Get the command from the robot
        cmd = self.GET_robot_command()
        # Takes action based on command
        self._take_action(cmd)
        # Sends updates if workspace has changed
        self._perceptual_update()

    def _bootstrap(self):
        "Sends OntoSem initial 'bootstrap' info about workspace"
        bootstrap_dict = {}
        # We assume that initially that the workspace is consistent with the launchfile
        bootstrap_dict.update(rospy.get_param("action_provider/objects_left"))
        bootstrap_dict.update(rospy.get_param("action_provider/objects_right"))

        faces                   = rospy.get_param(lf.param_prefix + '/all_faces')
        bootstrap_dict['faces'] = faces

        rospy.loginfo(bootstrap_dict)

        # NOTE: Uncomment when running OntoSem
        # self.POST_bootstrap_ontosem(bootstrap_dict)


    # TODO Implement
    def _take_action(self, cmd, callback_id):
        """
        Recieves actions in the form:
           {“speak”: “….“}
           {“get”: 123}
           {“hold”: “…”}
        and enacts them accordingly

        After an action has been successfully executed
        we must send a completion msg back to ontosem in
        the form:

        {
        “callback-id”: “EXE.CALLBACK.123"
        }`
        """

        spoken_flag = False
        while(self.LISTENING):
            if not spoken_flag:
                rospy.loginfo("Waiting until query is done....")
                spoken_flag = True

            rospy.sleep(0.1)

        self.curr_action = cmd

        # FIX BASED ON FORMAT OF CMD
        cmd, arm, obj = parse_action(cmd.name, self.OBJECT_DICT)

        # TODO: keep track of arm usage to avoid using both at once

        elapsed_time = time.time() - self.strt_time
        rospy.loginfo(
            "Taking action {} on object {} with {} arm at time {}".format(cmd,
                                                                          obj,
                                                                          arm_str,
                                                                          elapsed_time))
        # Send action to the robot
        self._action(arm, (cmd, [obj]), {'wait': False})

        elapsed_time = time.time() - self.strt_time

        rospy.loginfo(
            "Took action {} on object {} with {} arm at time {}".format(cmd,
                                                                        obj,
                                                                        arm_str,
                                                                        elapsed_time))

        # TODO Uncomment when running OntoSem
        #self.POST_completed_action(act_dict)

    def _get_faces(self):
        """
        Tries to identify a face from the video stream for 3 seconds
        returns: list of strings (could be empty) corresponding to faces recognized
        """
        self.goal.order_id       = self.RECOGNIZE_ONCE
        self.goal.order_argument = self.DUMMY_STRING

        # The face_recognition package uses actionlib (http://wiki.ros.org/actionlib) to send
        # messages, which allows for these optional callback  to be passed

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

        self.client.send_goal(goal, active_cb=_active_cb, feedback_cb=_feedback_cb, done_cb = _done_cb)
        self.client.wait_for_result(rospy.Duration(3.0))

        names = self.client.get_result()

        if names:
            return names.names
        else:
            rospy.loginfo("No faces were found!")
            return ['']

    def _listen_query_cb(self, msg):
        """
         TODO: get verbal commands and POST to ontosem in the form
            {“type”: “LANGUAGE”, input: “…….“, source: “ENV.HUMAN.1”}
        """
        rospy.loginfo("QUERY RECEIVED: {}".format(msg.transcript))

        with self.lock:
            self.LISTENING = True

        # What to do here?
        cmd_dict = {}
        cmd_dict["type"]    = "LANGUAGE"
        cmd_dict["input"]   = msg.transcript
        cmd_dict["source"]  = "ENV.HUMAN.1"

        # TODO uncomment when running OntoSem
        # POST_verbal_command(self, command_dict)

        with self.lock:
            self.LISTENING = False