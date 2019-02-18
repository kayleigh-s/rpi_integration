import rospy
import actionlib

# Documentation here: http://wiki.ros.org/face_recognition
from face_recognition.msg import FaceRecognitionGoal, FaceRecognitionAction, FRClientGoal

from svox_tts.srv import Speech, SpeechRequest
from human_robot_collaboration.controller import BaseController
from rpi_integration.learner_utils import RESTOntoSemUtils, parse_action
from ros_speech2text.msg import transcript # message format for ros_speech2text

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
        spoken_flag = False
        if not self.autostart:
            if not spoken_flag:
                rospy.loginfo("Waiting to start...")
                spoken_flag = True

        # how to loop through this to run continuously?
        while(self.run):
            cmd = self.GET_robot_command()
            self._take_action(cmd)
            self._perceptual_update()

    def _bootstrap(self):
        "Sends OntoSem initial 'bootstrap' info about workspace"
        bootstrap_dict = {}

        # We assume that initially that the workspace is consistent with the launchfile
        bootstrap_dict.update(rospy.get_param("action_provider/objects_left"))
        bootstrap_dict.update(rospy.get_param("action_provider/objects_right"))

        faces                   = rospy.get_param(lf.param_prefix + '/all_faces')
        bootstrap_dict['faces'] = faces
        workspace_1 = {"id": "workspace-1", "type": "WORKSPACE", "objects": [objects_left],   "faces": faces}
        workspace_2 = {"id": "workspace-2", "type": "WORKSPACE", "objects": [objects_right],  "faces": faces}

        bootstrap_dict["locations"] = [workspace_1, workspace_2]

        rospy.loginfo(bootstrap_dict)

        # NOTE: Uncomment when running OntoSem
        # self.POST_bootstrap_ontosem(bootstrap_dict)


    # TODO Implement
    def _take_action(self, cmd):
        """
        Recieves actions in the form:
           {“speak”: “…“, "callback": "SELF.CALLBACK.1"}
        and enacts them accordingly

        recognized objects: dowel, seat, back, front-bracket, foot-bracket, back-bracket, top-backet, screwdriver

        After an action has been successfully executed
        we must send a completion msg back to ontosem in
        the form:

        {
        “callback-id”: “EXE.CALLBACK.123"
        }
        """

        spoken_flag = False
        # while(self.LISTENING):
        #     if not spoken_flag:
        #         rospy.loginfo("Waiting until query is done....")
        #         spoken_flag = True

        #     rospy.sleep(0.1)


        for key, value in cmd:

            if key == "speak":
                # TODO speak conditional
                # Value is sentence to speak
                # Should this be the BaseController method or taken directly from svox_tts?
                self.say(value)

            elif key != "callback":
                # key is action we are taking
                self.curr_action = key

                """
                Value for action is object id:
                Object ids below 100 are accessed with Left arm, else Right arm
                """
                if value >= 100:
                    arm = BaseController.LEFT
                else:
                    arm = BaseController.RIGHT

                self._action(arm, (cmd, [value]), {'wait': False})
                post_dict = {"callback-id": cmd["callback"]}

                # TODO uncomment when running OntoSem
                # self.POST_completed_action(post_dict)

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
