#!/usr/bin/env python

import rospy
import actionlib
import json
# Documentation here: http://wiki.ros.org/face_recognition
from face_recognition.msg import FaceRecognitionGoal, FaceRecognitionAction, FRClientGoal

from svox_tts.srv import Speech, SpeechRequest
from human_robot_collaboration.controller import BaseController
from rpi_integration.learner_utils import RESTOntoSemUtils, parse_action
from rpi_integration_msgs.srv import RobotCommand
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
        self.param_prefix      = "/rpi_integration"
        self.autostart         = rospy.get_param(self.param_prefix + '/autostart')
        self.use_stt           = rospy.get_param(self.param_prefix + '/use_stt', False)
        self.use_tts           = rospy.get_param(self.param_prefix + '/use_tts', True)
        self.use_ontosem_comms = rospy.get_param(self.param_prefix + '/use_ontosem_comms')
        self.use_face_rec      = rospy.get_param(self.param_prefix + '/use_face_rec')

        self.client            = actionlib.SimpleActionClient("face_recognition", FaceRecognitionAction)
        self.goal              = FaceRecognitionGoal()

        self.storage_1         = [] # For the purposes of this demo, storage_1 is empty
        self.storage_2         = rospy.get_param(self.param_prefix + "/objects_right").values()

        self.workspace         = [] # stores ids of retrieved objects in shared workspace.
        # Candidate for deletion
        # self.strt_time       = time.time()
        self.LISTENING         = False

        # Listens for speech commands from microphone
        self._listen_sub       = rospy.Subscriber(self.STT_TOPIC, #self.SPEECH_SERVICE,
                                                   transcript, self._listen_query_cb)
        self._cmd               = None # Will store the current command POSTed by OntoSem
        BaseController.__init__(
            self,
            use_left=True,
            use_right=True,
            use_stt=self.use_stt,
            use_tts=self.use_tts,
            recovery=False,
        )

        RESTOntoSemUtils.__init__(self)


        if self.use_face_rec:
            rospy.logwarn("Waiting for actionserver....")
            self.client.wait_for_server()
            rospy.logwarn("Action server ready!")


        self._bootstrap()

    @property
    def cmd(self):
        return self._cmd

    @cmd.setter
    def cmd(self, cmd):
        with self.lock:
            self._cmd = cmd

    def _perceptual_update(self):
        """
        POSTs the current contents of the workspace to OntoSem.
        """
        rospy.loginfo("Sending perceptual update...")
        visual_dict = {
            "storage-1": [o for o in self.storage_1 if not o in self.workspace], # objects in workspace are
            "storage-2": [o for o in self.storage_2 if not o in self.workspace], # no longer in storage
            "faces":     self._get_faces()
        }

        if self.use_ontosem_comms:
            self.POST_visible_objects(json.dumps(visual_dict))


    def _run(self):
        """
        Waits for commands from OntoSem, enacts them, and then send the appropriate
        updates back to OntoSem.
        """
        # spoken_flag = False
        # if not self.autostart:
        #     if not spoken_flag:
        #         rospy.loginfo("Waiting to start...")
        #         spoken_flag = True

        self.service = rospy.Service('/http_to_srv', RobotCommand, self._take_action)
        rospy.sleep(5.0)
        self.POST_verbal_command(json.dumps({"input": "Let's build a chair.", "source": "@ENV.HUMAN.1",  "type":"LANGUAGE"}))
        rospy.sleep(3.0)
        self.GET_debug()
        rospy.loginfo("Sent command!")
        # if self._take_action(self._cmd):
        #     self._percetual_update()

    #TODO: output needs to conform to desired format outlined below
    def _bootstrap(self):
        """
        Sends OntoSem initial 'bootstrap' info about workspace in the following format:
                {
                  "locations": [
                    {
                      "id": "workspace-1",
                      "type": "WORKSPACE",
                      "objects": [],
                      "faces": ["jake"]
                    }, {
                      "id": "storage-1",
                      "type": "STORAGE",
                      "objects": [
                        {
                          "id": 1,
                          "type": "screwdriver"
                        }, {
                          "id": 2,
                          "type": "front-bracket"
                        }, {
                          "id": 3,
                          "type": "foot-bracket"
                        }
                      ],
                      "faces": []
                    }, {
                      "id": "storage-2",
                      "type": "STORAGE",
                      "objects": [],
                      "faces": []
                    }
                  ]
                }
        Here storage-1 and -2 refer to the tables to the left and right of Baxter.
        """

        self.GET_bootstrap() # Initializes agent knowledge
        self.GET_start()    # Starts agent
        rospy.loginfo("Sending bootstrap info...")

        bootstrap_dict = {}

        # We assume that initially that the workspace is consistent with the launchfile

        objs_left   = []
        objs_right  = rospy.get_param(self.param_prefix + "/objects_right")

        faces                   = rospy.get_param(self.param_prefix + '/all_faces')

        workspace_1 = {"id": "workspace-1",
                        "type": "WORKSPACE", 
                        "objects": [], 
                        "faces": faces}

        storage_1   = {"id": "storage-1",
                        "type": "STORAGE",
                        "objects": objs_left,
                        "faces": []}

        storage_2   = {"id": "storage-2",
                        "type": "STORAGE",
                        "objects": [{"id":v, "type":k} for k, v in objs_right.items()],
                        "faces": []
                        }

        bootstrap_dict["locations"] = [workspace_1, storage_1, storage_2]

        rospy.loginfo(bootstrap_dict)

        if self.use_ontosem_comms:
            self.POST_bootstrap_ontosem(json.dumps(bootstrap_dict))


    # TODO Implement
    def _take_action(self, req):
        """
        Receives actions in the form:
            {"speak": "...", "callback": "SELF.CALLBACK.1"}

        and enacts them accordingly.

        recognized objects: dowel, seat, back, front-bracket, foot-bracket, back-bracket, top-backet, screwdriver

        After an action has been successfully executed
        we must send a completion msg back to ontosem in
        the form:

        {
        "callback-id": "EXE.CALLBACK.123"
        }
        """

        ARM_ID_VAL  = 100 # Value for action is object id:
                          # Object ids below 100 are accessed with Left arm, else Right arm



        rospy.loginfo("Received command...")
        cmd = {req.cmd: req.id, "callback": req.callback}
        rospy.loginfo("CMD: {}".format(cmd))

        for key, value in cmd:

            if key == "speak":
                # Value is sentence to speak
                # Should this be the BaseController method or taken directly from svox_tts?
                self.say(value)

            elif key == "get":
                # key is action we are taking
                self.curr_action = key

                if value >= ARM_ID_VAL:
                    arm     = self.action_left # A method for passing cmd to left arm
                else:
                    arm     = self.action_right # A method for passing cmd to left arm

                r = arm((cmd, [value]), {'wait': False})

                if r.success:
                    rospy.loginfo("Successfully retrieved obj {}".format(value))

                    with self.lock:
                        self.workspace.append(value) # object has been successfully brought to workspace

            elif key == "hold":
                # This may cause a problem later because there are two different types of holds
                # but it should be ok for now.
                 r = self.action_right("hold_leg", 0)

        post_dict = {"callback-id": cmd["callback"]} # To be sent back to ontosem

        if self.use_ontosem_comms:
            self.POST_completed_action(json.dumps(post_dict))

        # with self.lock:
        #     self._cmd = None

        self._perceptual_update()
        return RobotCommandResponse("Ok")

    def _get_faces(self):
        """
        Tries to identify a face from the video stream for 3 seconds
        returns: list of strings (could be empty) corresponding to faces recognized
        """
        if self.use_face_rec:
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

            self.client.send_goal(self.goal, active_cb=_active_cb, feedback_cb=_feedback_cb, done_cb=_done_cb)
            self.client.wait_for_result(rospy.Duration(3.0))

            names = self.client.get_result()

            if names:
                return names.names
            else:
                rospy.loginfo("No faces were found!")
                return ['']
        else:
            return rospy.get_param(self.param_prefix + '/all_faces')

    def _listen_query_cb(self, msg):
        """
         TODO: get verbal commands and POST to ontosem in the form
            {"type": "LANGUAGE", input: "...", source: "ENV.HUMAN.1"}
        """
        rospy.loginfo("QUERY RECEIVED: {}".format(msg.transcript))

        with self.lock:
            self.LISTENING = True

        # What to do here?
        cmd_dict = {}
        cmd_dict["type"]    = "LANGUAGE"
        cmd_dict["input"]   = msg.transcript
        cmd_dict["source"]  = "ENV.HUMAN.1"


        if self.use_ontosem_comms:
            self.POST_verbal_command(json.dumps(cmd_dict))

        with self.lock:
            self.LISTENING = False
