#!/usr/bin/env python
import rospy
from flask import abort, Flask, request
from rpi_integration_msgs.srv import RobotCommand


app  = Flask(__name__)
name = "http_to_srv"

rospy.wait_for_service(name)
server = rospy.ServiceProxy(name, RobotCommand)

@app.route("/robotcommand", methods=["POST"])
def getCommand():
    rospy.loginfo("Receiving command from OntoSem...")
    if not request.get_json():
        abort(400)

    else:
        cmd = request.json()
        # srv_req = RobotCommandRequest(cmd["cmd"], cmd["id"], cmd["callback"])
        server(cmd["cmd"], cmd["id"], cmd["callback"])
    return "OK"

if __name__ == '__main__':
    rospy.init_node(name)
