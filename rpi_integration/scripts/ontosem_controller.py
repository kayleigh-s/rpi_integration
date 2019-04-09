#!/usr/bin/env python

import rospy
from rpi_integration.ontosem_controller import OntoSemController


if __name__ == '__main__':

    controller = OntoSemController()
    controller.run()
    rospy.spin()
