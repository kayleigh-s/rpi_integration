import rospy
from rpi_integration.htm_controoler import HTMController

try:
    controller = HTMController()

    if not controller.testing:
        controller.run()

except rospy.ROSInterruptException, KeyboardInterrupt:
    pass
