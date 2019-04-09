#!/usr/bin/env python

import rospy
from rpi_integration.ontosem_controller import OntoSemController

rospy.loginfo("Running OntoSemController")
controller = OntoSemController()
controller.run()
rospy.spin()
