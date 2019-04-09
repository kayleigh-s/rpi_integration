#!/usr/bin/env python

import rospy
from rpi_integration.htm_controller import HTMController

try:
    rospy.loginfo("Running HTMController")
    controller = HTMController()

    if not controller.testing:
        controller.run()

except rospy.ROSInterruptException, KeyboardInterrupt:
    pass
