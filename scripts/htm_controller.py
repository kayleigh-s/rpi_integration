import rospy
from rpi_integration.htm_controoler import HTMController

try:
    controller = HTMController()

    if not rospy.get_param('/rpi_integretion/testing', False):
        controller.run()

except rospy.ROSInterruptException, KeyboardInterrupt:
    pass
